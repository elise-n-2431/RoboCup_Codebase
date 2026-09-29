#include "pursuit_controller.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "outputs/smart_servo.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "debug_print.h"
#include "path_finding.h"
#include "pose.h"
#include "navigation/reversing_controller.h"


// Pursuit tuning
static const int WEIGHT_DETECT_DISTANCE_MM = 650;
static const int WEIGHT_MIDDLE_SENSOR = 8;

static const int MIDDLE_LOST_COUNT_REQUIRED = 3;

static const float PURSUIT_SCAN_STEP_DEG = 15.0f;
static const float PURSUIT_SCAN_MAX_DEG = 60.0f;
static const float GEO_SCAN_STEP_DEG = 5.0f;
static const float GEO_SCAN_MAX_DEG = 10.0f;
static const int GEO_CENTRE_MAX_MM = 600;
static const int GEO_CENTRE_SAMPLES_REQUIRED = 2;

// Robot frame: +x forward, +y left, origin at the ICR.
// The angled beams cross the centreline about 300 mm ahead of the centre ToF.
static const float GEO_CENTRE_SENSOR_X_MM = 120.0f;
static const float GEO_SIDE_FORWARD_OFFSET_MM = 110.0f;
static const float GEO_SIDE_SENSOR_X_MM =
    GEO_CENTRE_SENSOR_X_MM + GEO_SIDE_FORWARD_OFFSET_MM;
static const float GEO_SIDE_SENSOR_Y_MM = 90.0f;
static const float GEO_INWARD_ANGLE_DEG = 25.35f;
static const float GEO_LEFT_BEARING_BIAS_DEG = 0.0f;
static const float GEO_RIGHT_BEARING_BIAS_DEG = 0.0f;
static const float GEO_DIRECT_SCAN_THRESHOLD_DEG = 6.0f;

static const int WEIGHT_STOP_DISTANCE_MM = 80;
static const int WEIGHT_ENTRANCE_SAMPLES_REQUIRED = 2;
static const float GEO_CLOSE_LOCK_APPROACH_MM = 15.0f;
static const int WEIGHT_SLOW_DISTANCE_MM = 200;

static const int WEIGHT_APPROACH_POWER = 450;
static const int WEIGHT_SLOW_POWER = 420;
static const int PURSUIT_WALL_ABORT_MM = 25;
static const int PURSUIT_BROAD_OBSTACLE_MM = 250;
static const int PURSUIT_NAV_AGREEMENT_MM = 80;
static const int PURSUIT_WALL_BEHIND_NAV_MM = 50;
static const int PURSUIT_NARROW_WALL_OFFSET_MM = 150;
static const unsigned long PURSUIT_SECURE_SAMPLE_TIMEOUT_MS = 600;
static const int PURSUIT_STUCK_FRONT_MM = 80;
static const int PURSUIT_STUCK_PROGRESS_MM = 25;
static const unsigned long PURSUIT_STUCK_TIME_MS = 900;

static const unsigned long ARM_SECURE_WAIT_MS = 200;
static const unsigned long PURSUIT_TIMEOUT_MS = 10000;

int target_x_mm = 0;
int target_y_mm = 0;

enum PursuitState
{
    PURSUIT_START,
    PURSUIT_GEO_TURNING,
    PURSUIT_TURNING,
    PURSUIT_ACQUIRING,
    PURSUIT_APPROACHING,
    PURSUIT_SECURING,
    PURSUIT_FINISHED
};

static PursuitState pursuitState = PURSUIT_START;
static WeightTargetSide weightTargetSide = TARGET_NONE;

static int middleLostCount = 0;
static int geoRangeMm = 0;
static int geoCentreEvidenceCount = 0;

static float weightApproachHeading = 0.0f;
static bool weightApproachSlowed = false;

static float pursuitEntryHeading = 0.0f;
static float pursuitScanOriginHeading = 0.0f;
static float geoPredictedHeading = 0.0f;
static float geoRequestedTurnDeg = 0.0f;

static int pursuitScanIndex = 0;
static int pursuitPreferredDirection = 1;

static uint32_t lastMiddleSample = 0;

static unsigned long pursuitSecureStartedAt = 0;
static unsigned long pursuitStartedAt = 0;
static int entranceEvidenceCount = 0;
static int lastCentreApproachDistance = 0;
static int centreClosingCount = 0;
static bool geoCloseLockRequiresApproach = false;
static float approachStartX = 0.0f;
static float approachStartY = 0.0f;
static float approachStartHeadingRad = 0.0f;
static int pursuitProgressReferenceMm = 0;
static unsigned long pursuitProgressAt = 0;

static float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//needs this as uses middle sensor to hone in 
static bool readFreshMiddleDistance(int &distance)
{
    uint32_t sample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);

    if (sample == lastMiddleSample)
    {
        return false;
    }

    lastMiddleSample = sample;
    distance = tof_get_weight_middle();

    return true;
}

static void resetPursuitScan()
{
    pursuitScanOriginHeading = imu_get_heading();
    pursuitScanIndex = 0;

    // Ignore the current reading. Wait for a genuinely new sample.
    lastMiddleSample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);
}

static bool commandNextPursuitScan()
{
    bool geometricScan = weightTargetSide != TARGET_CENTRE;
    float scanStep = geometricScan ? GEO_SCAN_STEP_DEG : PURSUIT_SCAN_STEP_DEG;
    float scanMax = geometricScan ? GEO_SCAN_MAX_DEG : PURSUIT_SCAN_MAX_DEG;
    int stepsPerSide =
        (int)(
            scanMax /
            scanStep
        );

    if (pursuitScanIndex >=
        stepsPerSide * 2)
    {
        return false;
    }

    int level =
        pursuitScanIndex / 2 + 1;

    int direction =
        (pursuitScanIndex % 2 == 0)
        ? pursuitPreferredDirection
        : -pursuitPreferredDirection;

    float offset =
        direction *
        level *
        scanStep;

    pursuitScanIndex++;

    if (geometricScan)
    {
        float relativeTurn = wrap180(
            pursuitScanOriginHeading + offset - imu_get_heading());
        debugNav.print("GEO_SCAN,");
        debugNav.print(offset);
        debugNav.print(",");
        debugNav.println(relativeTurn);
        motor_control_turn_relative(relativeTurn);
        return true;
    }

    debugNav.print(
        "Pursuit: scanning offset "
    );
    debugNav.print(offset);
    debugNav.println(" deg");

    motor_control_turn_to(
        pursuitScanOriginHeading +
        offset
    );

    return true;
}

void pursuit_start(WeightTargetSide target)
{
    weightTargetSide = target;
    geoRangeMm = target == TARGET_LEFT ? tof_get_weight_left_bottom()
               : target == TARGET_RIGHT ? tof_get_weight_right_bottom() : 0;
    pursuitPreferredDirection = 1;
    pursuitState = PURSUIT_START;

    middleLostCount = 0;
    geoCentreEvidenceCount = 0;
    weightApproachSlowed = false;

    pursuitScanIndex = 0;
    pursuitSecureStartedAt = 0;
    pursuitProgressAt = 0;

    pursuitStartedAt = millis();
}

static int pursuitClearanceValue(
    int distance)
{
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}

static float targetDistance()
{
    float dx = get_frontier_world_x_mm() - pose_get_x_mm();
    float dy = get_frontier_world_y_mm() - pose_get_y_mm();

    return sqrtf(dx * dx + dy * dy);
}


static void beginCentreApproach(int centreDistance, bool fromGeo = false)
{
    debugNav.print("Pursuit: centre acquired weight at ");
    debugNav.print(centreDistance);
    debugNav.println(" mm");
    debugNav.println("PURSUIT_PHASE,CENTRE_APPROACH");

    middleLostCount = 0;
    weightApproachHeading = imu_get_heading();
    weightApproachSlowed = false;
    lastCentreApproachDistance = centreDistance;
    pursuitProgressReferenceMm = centreDistance;
    pursuitProgressAt = millis();
    centreClosingCount = 0;
    entranceEvidenceCount = 0;
    geoCloseLockRequiresApproach = fromGeo && centreDistance <= 80;
    approachStartX = pose_get_x_mm();
    approachStartY = pose_get_y_mm();
    approachStartHeadingRad = pose_get_heading_deg() * PI / 180.0f;
    pursuitState = PURSUIT_APPROACHING;
}

static float forwardApproachMm()
{
    return (pose_get_x_mm() - approachStartX) * cosf(approachStartHeadingRad) +
           (pose_get_y_mm() - approachStartY) * sinf(approachStartHeadingRad);
}

static void abortSecure(const char *reason)
{
    pursuitProgressAt = 0;
    motor_control_stop();
    smartservo_arms_open();
    debugNav.print("PURSUIT_SECURE_ABORT,");
    debugNav.println(reason);
    setStateFlag(&STATE_FLAGS.target_lost);
    pursuitState = PURSUIT_FINISHED;
}

static void abortGeometry(const char *reason)
{
    pursuitProgressAt = 0;
    motor_control_stop();
    weight_detection_reset();
    debugNav.print("GEO_ABORT,");
    debugNav.println(reason);
    setStateFlag(&STATE_FLAGS.target_lost);
    pursuitState = PURSUIT_FINISHED;
}

static void abortPursuitObstacle(int innerLeft, int innerRight, int middle)
{
    pursuitProgressAt = 0;
    motor_control_stop();
    debugNav.print("PURSUIT_OBSTACLE_ABORT,");
    debugNav.print(innerLeft);
    debugNav.print(",");
    debugNav.print(innerRight);
    debugNav.print(",");
    debugNav.println(middle);
    reversing_set_reason(REVERSE_CRITICAL_OBSTACLE);
    setStateFlag(&STATE_FLAGS.target_lost);
    setStateFlag(&STATE_FLAGS.reverse_triggered);
    pursuitState = PURSUIT_FINISHED;
}

FLASHMEM static bool checkGeoCentre(bool &newSample, int &evidenceCount)
{
    int centreDistance = -1;
    newSample = readFreshMiddleDistance(centreDistance);
    if (!newSample) return false;

    // The side pair already established a weight candidate. GEO only
    // needs two fresh plausible centre returns to hand off to pursuit.
    geoCentreEvidenceCount =
        centreDistance > 0 && centreDistance <= GEO_CENTRE_MAX_MM
        ? geoCentreEvidenceCount + 1 : 0;
    evidenceCount = geoCentreEvidenceCount;

    debugNav.print("GEO_CENTER_CHECK,");
    debugNav.print(centreDistance);
    debugNav.print(",");
    debugNav.println(evidenceCount);

    if (evidenceCount < GEO_CENTRE_SAMPLES_REQUIRED) return false;

    float scanOffset = wrap180(
        imu_get_heading() - pursuitScanOriginHeading);
    debugNav.print("GEO_LOCK,");
    debugNav.print(centreDistance);
    debugNav.print(",");
    debugNav.println(scanOffset);
    debugNav.print("GEO_LOCK_ERROR,");
    debugNav.println(wrap180(imu_get_heading() - geoPredictedHeading));

    motor_control_stop();
    weight_detection_reset();
    pursuitPreferredDirection = weightTargetSide == TARGET_LEFT ? -1 : 1;
    weightTargetSide = TARGET_CENTRE;
    lastMiddleSample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);
    pursuitStartedAt = millis();
    beginCentreApproach(centreDistance, true);
    return true;
}

void pursuit_update()
{   
    if (pursuitStartedAt != 0 &&
        pursuitState != PURSUIT_SECURING &&
        pursuitState != PURSUIT_FINISHED &&
        millis() - pursuitStartedAt >
            PURSUIT_TIMEOUT_MS)
    {
        if (weightTargetSide != TARGET_CENTRE)
        {
            abortGeometry("TARGET_LOST");
            return;
        }

        motor_control_stop();

        debugNav.println(
            "Pursuit: timeout - target lost"
        );

        setStateFlag(
            &STATE_FLAGS.target_lost
        );

        pursuitState =
            PURSUIT_FINISHED;

        return;
    }
    switch (pursuitState)
    {
        case PURSUIT_START:
        {
            motor_control_stop();
            // Open funnel before attempting to line up.
            smartservo_arms_open();

            pursuitEntryHeading = imu_get_heading();

            if (weightTargetSide == TARGET_LEFT)
            {
                pursuitPreferredDirection = -1;
                debugNav.println("Pursuit: target came from LEFT");
            }
            else if (weightTargetSide == TARGET_RIGHT)
            {
                pursuitPreferredDirection = 1;
                debugNav.println("Pursuit: target came from RIGHT");
            }
            else if (weightTargetSide == TARGET_CENTRE)
            {
                debugNav.println("Pursuit: target detected directly ahead");
            }
            else
            {
                debugNav.println("Pursuit: started without target side");

                setStateFlag(&STATE_FLAGS.target_lost);
                pursuitState = PURSUIT_FINISHED;
                return;
            }

            debugNav.print("Pursuit: search origin heading ");
            debugNav.println(pursuitEntryHeading);

            if (weightTargetSide == TARGET_CENTRE)
            {
                resetPursuitScan();
                pursuitState = PURSUIT_ACQUIRING;
                break;
            }

            if (geoRangeMm <= 0 || geoRangeMm > WEIGHT_DETECT_DISTANCE_MM)
            {
                abortGeometry("INVALID_RANGE");
                return;
            }

            float sensorY = weightTargetSide == TARGET_LEFT
                ? GEO_SIDE_SENSOR_Y_MM : -GEO_SIDE_SENSOR_Y_MM;
            float sensorAngleDeg = weightTargetSide == TARGET_LEFT
                ? -GEO_INWARD_ANGLE_DEG : GEO_INWARD_ANGLE_DEG;
            float sensorAngleRad = sensorAngleDeg * PI / 180.0f;
            float hitX = GEO_SIDE_SENSOR_X_MM +
                         geoRangeMm * cosf(sensorAngleRad);
            float hitY = sensorY + geoRangeMm * sinf(sensorAngleRad);
            float bearingDeg = atan2f(hitY, hitX) * 180.0f / PI;
            bearingDeg += weightTargetSide == TARGET_LEFT
                ? GEO_LEFT_BEARING_BIAS_DEG
                : GEO_RIGHT_BEARING_BIAS_DEG;
            float relativeTurn = -bearingDeg;
            geoRequestedTurnDeg = relativeTurn;
            geoPredictedHeading = pursuitEntryHeading + relativeTurn;

            debugNav.print("GEO_ACQUIRE,");
            debugNav.print(weightTargetSide == TARGET_LEFT ? "LEFT," : "RIGHT,");
            debugNav.print(geoRangeMm);
            debugNav.print(",");
            debugNav.print(hitX);
            debugNav.print(",");
            debugNav.print(hitY);
            debugNav.print(",");
            debugNav.println(bearingDeg);
            if (fabsf(bearingDeg) <= GEO_DIRECT_SCAN_THRESHOLD_DEG)
            {
                debugNav.print("GEO_TURN_SKIPPED_SMALL,");
                debugNav.println(bearingDeg);
                resetPursuitScan();
                pursuitState = PURSUIT_ACQUIRING;
                debugNav.println("PURSUIT_PHASE,GEO_SCAN");
                break;
            }
            debugNav.print("GEO_TURN_START,");
            debugNav.print(bearingDeg);
            debugNav.print(",");
            debugNav.println(relativeTurn);
            debugNav.print("GEO_TURN_REQUEST,");
            debugNav.println(bearingDeg);
            debugNav.println("PURSUIT_PHASE,GEO_TURN");

            // Pose/robot-left angles and IMU/motor turns have opposite signs.
            motor_control_turn_relative_geo(relativeTurn);
            pursuitState = PURSUIT_GEO_TURNING;
            break;
        }

        case PURSUIT_GEO_TURNING:
        {
            if (motor_control_is_turning()) return;

            motor_control_stop();
            debugNav.print("GEO_TURN_COMPLETE,");
            debugNav.print(geoRequestedTurnDeg);
            debugNav.print(",");
            debugNav.println(wrap180(imu_get_heading() - pursuitEntryHeading));
            pursuitPreferredDirection = 1;
            resetPursuitScan();
            pursuitScanOriginHeading = geoPredictedHeading;
            pursuitState = PURSUIT_ACQUIRING;
            debugNav.println("PURSUIT_PHASE,GEO_SCAN");
            break;
        }

        case PURSUIT_TURNING:
        {
            if (weightTargetSide != TARGET_CENTRE)
            {
                bool newSample = false;
                int evidenceCount = 0;
                if (checkGeoCentre(newSample, evidenceCount)) return;
            }
            else
            {
                int centreDistance;

                if (readFreshMiddleDistance(centreDistance))
                {
                    if (centreDistance > 0 &&
                        centreDistance <= WEIGHT_DETECT_DISTANCE_MM)
                    {
                        debugNav.print("Pursuit: centre found during scan at ");
                        debugNav.print(centreDistance);
                        debugNav.println(" mm");

                        motor_control_stop();

                        // Check again once stationary before committing.
                        pursuitState = PURSUIT_ACQUIRING;
                        return;
                    }
                }
            }

            if (motor_control_is_turning())
            {
                return;
            }

            motor_control_stop();
            debugNav.println("Pursuit: scan turn complete");

            lastMiddleSample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);

            pursuitState = PURSUIT_ACQUIRING;
            break;
        }

        case PURSUIT_ACQUIRING:
        {
            if (weightTargetSide != TARGET_CENTRE)
            {
                bool newSample = false;
                int evidenceCount = 0;
                if (checkGeoCentre(newSample, evidenceCount)) return;
                if (!newSample || evidenceCount > 0) return;

                if (!commandNextPursuitScan())
                {
                    abortGeometry("SCAN_EXHAUSTED");
                    return;
                }

                pursuitState = PURSUIT_TURNING;
                break;
            }

            int centreDistance;

            // Only make one decision per actual ToF measurement.
            if (!readFreshMiddleDistance(centreDistance))
            {
                return;
            }

            // Centre has found the weight.
            if (centreDistance > 0 &&
                centreDistance <= WEIGHT_DETECT_DISTANCE_MM)
            {
                beginCentreApproach(centreDistance);
                return;
            }

            // Centre still cannot see it.
            if (!commandNextPursuitScan())
            {
                motor_control_stop();
                debugNav.println("Pursuit: scan exhausted - target lost");

                setStateFlag(&STATE_FLAGS.target_lost);
                pursuitState = PURSUIT_FINISHED;
                return;
            }

            pursuitState = PURSUIT_TURNING;
            break;
        }

        case PURSUIT_APPROACHING:
        {
            int innerLeft = tof_get_nav_inner_left();
            int innerRight = tof_get_nav_inner_right();
            int middleNow = tof_get_weight_middle();
            bool atEntrance = middleNow > 0 && middleNow <= WEIGHT_STOP_DISTANCE_MM;
            bool broadObstacle =
                innerLeft > 0 && innerRight > 0 &&
                innerLeft <= PURSUIT_BROAD_OBSTACLE_MM &&
                innerRight <= PURSUIT_BROAD_OBSTACLE_MM &&
                abs(innerLeft - innerRight) <= PURSUIT_NAV_AGREEMENT_MM &&
                (middleNow <= 0 || middleNow >=
                    (innerLeft + innerRight) / 2 + PURSUIT_WALL_BEHIND_NAV_MM);

            int closestNav = min(pursuitClearanceValue(innerLeft),
                                 pursuitClearanceValue(innerRight));
            // A single close nav return can be the centred weight's edge.
            // Keep the hard stop when centre range is not closing or a wall
            // is substantially closer than the centre target.
            bool narrowObstacle = closestNav <= PURSUIT_WALL_ABORT_MM &&
                (centreClosingCount < 2 || middleNow <= 0 ||
                 middleNow >= closestNav + PURSUIT_NARROW_WALL_OFFSET_MM);

            if (!atEntrance && (broadObstacle || narrowObstacle))
            {
                abortPursuitObstacle(innerLeft, innerRight, middleNow);
                return;
            }

            int centreDistance;

            // Only react to genuinely new middle-ToF samples.
            if (!readFreshMiddleDistance(centreDistance))
            {
                return;
            }
            // Lost target.
            if (centreDistance <= 0 ||
                centreDistance > WEIGHT_DETECT_DISTANCE_MM)
            {
                middleLostCount++;

                debugNav.print("Pursuit: centre miss ");
                debugNav.print(middleLostCount);
                debugNav.print("/");
                debugNav.println(MIDDLE_LOST_COUNT_REQUIRED);

                if (middleLostCount >= MIDDLE_LOST_COUNT_REQUIRED)
                {
                    motor_control_stop();

                    weightApproachSlowed = false;
                    middleLostCount = 0;

                    debugNav.println("Pursuit: centre lost weight - reacquiring");

                    resetPursuitScan();
                    pursuitState = PURSUIT_ACQUIRING;
                }

                return;
            }

            middleLostCount = 0;
            if (centreDistance <= pursuitProgressReferenceMm -
                                  PURSUIT_STUCK_PROGRESS_MM)
            {
                pursuitProgressReferenceMm = centreDistance;
                pursuitProgressAt = millis();
            }
            if (centreDistance > WEIGHT_STOP_DISTANCE_MM &&
                motor_control_is_driving() &&
                closestNav < PURSUIT_STUCK_FRONT_MM &&
                pursuitProgressAt != 0 &&
                millis() - pursuitProgressAt >= PURSUIT_STUCK_TIME_MS)
            {
                debugNav.print("PURSUIT_STUCK_NO_RANGE_PROGRESS,");
                debugNav.print(centreDistance);
                debugNav.print(",");
                debugNav.print(closestNav);
                debugNav.print(",");
                debugNav.println(millis() - pursuitProgressAt);
                abortPursuitObstacle(innerLeft, innerRight, centreDistance);
                return;
            }
            if (centreDistance < lastCentreApproachDistance - 5)
                centreClosingCount = min(centreClosingCount + 1, 2);
            else if (centreDistance > lastCentreApproachDistance + 10)
                centreClosingCount = 0;
            lastCentreApproachDistance = centreDistance;

            // Weight has reached the funnel.
            if (centreDistance <= WEIGHT_STOP_DISTANCE_MM)
            {
                entranceEvidenceCount++;
                float approachMm = forwardApproachMm();
                if (entranceEvidenceCount >= WEIGHT_ENTRANCE_SAMPLES_REQUIRED &&
                    (!geoCloseLockRequiresApproach ||
                     approachMm >= GEO_CLOSE_LOCK_APPROACH_MM))
                {
                    debugNav.print("PURSUIT_ENTRANCE_CONFIRM,");
                    debugNav.print(centreDistance);
                    debugNav.print(",");
                    debugNav.println(approachMm);
                    motor_control_stop();

                    debugNav.print("Pursuit: weight reached entrance at ");
                    debugNav.print(centreDistance);
                    debugNav.println(" mm");

                    // Secure weight before SORTING takes control.
                    smartservo_arms_close();

                    pursuitSecureStartedAt = millis();
                    pursuitState = PURSUIT_SECURING;
                    pursuitProgressAt = 0;
                    return;
                }
            }
            else entranceEvidenceCount = 0;

            // Slow approach.
            if (centreDistance <= WEIGHT_SLOW_DISTANCE_MM)
            {
                if (!weightApproachSlowed ||
                    !motor_control_is_driving())
                {
                    motor_control_drive_heading(
                        weightApproachHeading,
                        WEIGHT_SLOW_POWER
                    );

                    weightApproachSlowed = true;
                    debugNav.println("Pursuit: slowing approach");
                }

                return;
            }

            // Normal approach.
            if (!motor_control_is_driving())
            {
                motor_control_drive_heading(
                    weightApproachHeading,
                    WEIGHT_APPROACH_POWER
                );
            }

            break;
        }

        case PURSUIT_SECURING:
        {
            motor_control_stop();

            if (millis() - pursuitSecureStartedAt < ARM_SECURE_WAIT_MS)
            {
                return;
            }

            int securedDistance;
            if (!readFreshMiddleDistance(securedDistance))
            {
                if (millis() - pursuitSecureStartedAt >=
                    PURSUIT_SECURE_SAMPLE_TIMEOUT_MS)
                {
                    abortSecure("NO_FRESH_CENTRE");
                }
                return;
            }
            if (securedDistance <= 0 ||
                securedDistance > WEIGHT_STOP_DISTANCE_MM)
            {
                abortSecure("CENTRE_NOT_AT_ENTRANCE");
                return;
            }

            debugNav.println("Pursuit: weight secured");

            pursuitState = PURSUIT_FINISHED;
            pursuitProgressAt = 0;
            setStateFlag(&STATE_FLAGS.weight_in_entrance);

            break;
        }

        case PURSUIT_FINISHED:
        {
            // Waiting for the state machine to take over.
            break;
        }
    }
}

void pursuit_reset()
{
    pursuitState = PURSUIT_START;
    weightTargetSide = TARGET_NONE;

    middleLostCount = 0;
    geoCentreEvidenceCount = 0;
    geoRangeMm = 0;

    weightApproachHeading = 0.0f;
    weightApproachSlowed = false;

    pursuitEntryHeading = 0.0f;
    pursuitScanOriginHeading = 0.0f;
    geoPredictedHeading = 0.0f;
    geoRequestedTurnDeg = 0.0f;
    pursuitScanIndex = 0;

    pursuitSecureStartedAt = 0;
    pursuitProgressAt = 0;
    pursuitStartedAt = 0;
}
