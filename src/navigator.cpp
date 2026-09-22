#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "inputs/tof_expander.h"
#include "state_machine.h"
#include "outputs/smart_servo.h"
#include "map.h"
#include "pose.h"
#include "arena_config.h"
#include "debug_print.h"


static const int WEIGHT_DETECT_DISTANCE_MM = 550;
static const int WEIGHT_MIDDLE_SENSOR = 8;

static int middleLostCount = 0;
static const int MIDDLE_LOST_COUNT_REQUIRED = 3;


// Pursuit no longer assumes that a side detection means a fixed 36 degree turn.
// Instead, search around the heading where pursuit started until the centre
// ToF genuinely sees the weight.
static const float PURSUIT_SCAN_STEP_DEG = 12.0f;
static const float PURSUIT_SCAN_MAX_DEG  = 48.0f;


static const int WEIGHT_DIFFERENCE_MM = 100;

static const int WEIGHT_STOP_DISTANCE_MM = 100;

static const int WEIGHT_SLOW_DISTANCE_MM = 200;

static const int WEIGHT_APPROACH_POWER = 360;
static const int WEIGHT_SLOW_POWER = 300;


static const int WEIGHT_LEFT_TOP_SENSOR     = 3;
static const int WEIGHT_LEFT_BOTTOM_SENSOR  = 4;
static const int WEIGHT_RIGHT_TOP_SENSOR    = 2;
static const int WEIGHT_RIGHT_BOTTOM_SENSOR = 1;
static const unsigned long WEIGHT_EVIDENCE_TIMEOUT_MS = 450;


static int leftDetectionCount = 0;
static int rightDetectionCount = 0;
static float weightApproachHeading = 0.0f;
static bool weightApproachSlowed = false;

static int middleDetectionCount = 0;
static uint32_t lastMiddleDetectionSample = 0;

static const int SIDE_DETECTION_COUNT_REQUIRED = 2;
static const int MIDDLE_DETECTION_COUNT_REQUIRED = 2;
static const unsigned long WEIGHT_RETRIGGER_BLOCK_MS = 1500;
static unsigned long weightDetectionBlockedUntil = 0;

static const int CENTRE_NAV_WALL_OFFSET_MM = 150;

// Allow mounting / wall-angle / sensor noise variation.
static const int CENTRE_NAV_WALL_TOLERANCE_MM = 75;

// A broad wall should normally be seen at roughly the same
// distance by both inner navigation sensors.
static const int INNER_WALL_AGREEMENT_MM = 120;

// Heading before the robot diverted from roaming toward this weight.
// Used later if REVERSING needs to restore the original direction.
static float pursuitEntryHeading = 0.0f;


// Heading around which the current centre-acquisition scan is performed.
static float pursuitScanOriginHeading = 0.0f;


// Which scan target we are up to.
static int pursuitScanIndex = 0;


// -1 means search left first, +1 means search right first.
static int pursuitPreferredDirection = 1;


// Used so one stale centre-ToF reading cannot be counted many times
// by the much faster main loop.
static uint32_t lastMiddleSample = 0;


//In the roaming state to see which side it thinks the weight is on
enum WeightTargetSide
{
    TARGET_NONE,
    TARGET_LEFT,
    TARGET_RIGHT,
    TARGET_CENTRE
};

static WeightTargetSide weightTargetSide = TARGET_NONE;


enum PursuitState
{
    PURSUIT_START,
    PURSUIT_TURNING,
    PURSUIT_ACQUIRING,
    PURSUIT_APPROACHING,
    PURSUIT_SECURING,
    PURSUIT_FINISHED
};

static PursuitState pursuitState = PURSUIT_START;

enum ReversingState
{
    REVERSE_START,
    REVERSE_BACKING,
    REVERSE_TURNING,
    REVERSE_FINISHED
};

static unsigned long pursuitSecureStartedAt = 0;

static const unsigned long ARM_SECURE_WAIT_MS = 1000;

static ReversingState reversingState = REVERSE_START;

enum RoamingState
{
    ROAM_START,
    ROAM_DRIVING,
    ROAM_TURNING,
    ROAM_CHECKING
};
static RoamingState roamingState = ROAM_START;

static const float REVERSE_ESCAPE_TURN_DEG = 60.0f;

static float flatPitchReference = 0.0f;

static unsigned long pitchExceededAt = 0;
static unsigned long rampDetectionBlockedUntil = 0;

static bool reverseTriggeredByPitch = false;

static const float RAMP_TRIGGER_DEG = 10.0f;

// Consider ourselves back on level ground around here.
static const float RAMP_RELEASE_DEG = 5.0f;

// Pitch must remain abnormal for this long before triggering.
static const unsigned long RAMP_CONFIRM_MS = 200;

// Ramp reverse is different from dummy reverse:
// reverse at least this long...
static const unsigned long RAMP_MIN_REVERSE_MS = 600;

// ...but never sit reversing forever.
static const unsigned long RAMP_MAX_REVERSE_MS = 2500;


// ============================================================
// ROAMING TUNING
// ============================================================

static int ROAM_POWER = 430;

// Distance at which normal avoidance begins
static int ROAM_FRONT_BLOCK_MM = 250;

static int ROAM_CRITICAL_MM = 90;

// Normal avoidance turn
static float ROAM_AVOID_TURN_DEG = 45.0f;

// More aggressive recovery turn later if needed
static float ROAM_RECOVERY_TURN_DEG = 90.0f;

static int ROAM_REVERSE_POWER = 250;
static unsigned long ROAM_REVERSE_TIME_MS = 500;

static const int REVERSE_POWER = 250;
static const unsigned long REVERSE_TIME_MS = 2000;
//to time when the reverse started so it knows when to count 1 second from
static unsigned long reverseStartedAt = 0;

static const int ROAM_SIDE_BLOCK_MM = 180;
static const int ROAM_SLOW_MM = 500;
static const int ROAM_SLOW_POWER = 340;

static const unsigned long NAV_TURN_TIMEOUT_MS = 8000;
static const unsigned long PURSUIT_TIMEOUT_MS = 10000;

static bool navigatorEnabled = true;
static bool roamingPickupEnabled = false;
static bool turnWatchActive = false;

static NavState lastNavState = STATIONARY;

static unsigned long navigatorStateStarted = 0;
static unsigned long turnStartedAt = 0;
static unsigned long roamCheckStartedAt = 0;
static unsigned long lastWeightCheckAt = 0;

static float roamHeading = 0.0f;
static int roamCommandedPower = 0;
static int roamTurnDirection = 1;

enum HomingState
{
    HOMING_START,
    HOMING_TURNING,
    HOMING_DRIVING,
    HOMING_AVOIDING,
    HOMING_DOCKING,
    HOMING_DOCK_TURNING
};

static HomingState homingState = HOMING_START;

static const int HOME_POWER = 430;
static const int HOME_SLOW_POWER = 340;

static const float HOME_SLOW_DISTANCE_MM = 700.0f;

static const int HOME_FRONT_BLOCK_MM = 250;

static const float HOME_AVOID_TURN_DEG = 45.0f;

static unsigned long lastHomeHeadingUpdate = 0;

static const unsigned long HOME_HEADING_UPDATE_MS = 250;

static unsigned long homeDockStart = 0;

static const int HOME_DOCK_POWER = 280;
static const unsigned long HOME_DOCK_TIME_MS = 1800;



static float wrap180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float homeDistance()
{
    float dx =
        arena_get_home_x_mm() - pose_get_x_mm();

    float dy =
        arena_get_home_y_mm() - pose_get_y_mm();

    return sqrtf(
        dx * dx +
        dy * dy
    );
}

static float homeHeadingError()
{
    float dx =
        arena_get_home_x_mm() - pose_get_x_mm();

    float dy =
        arena_get_home_y_mm() - pose_get_y_mm();

    // Heading in the pose coordinate system
    float desiredPoseHeading =
        atan2f(dy, dx)
        * 180.0f / PI;

    float currentPoseHeading =
        pose_get_heading_deg();

    return wrap180(
        desiredPoseHeading -
        currentPoseHeading
    );
}

static bool weightPairDetected(int top, int bottom, int navDistance, int &difference)
{
    difference = 0;

    if (top < 0 || bottom <= 0 || bottom > WEIGHT_DETECT_DISTANCE_MM) return false;
    if (top == 0) {
        if (bottom > 330)
        {
            return false;
        }

        // If the nav sensor sees something at approximately
        // the same distance, it is probably a wall.
        if (navDistance > 0 && abs(navDistance - bottom) < 100)
        {
            return false;
        }
        return true;
    }
    difference = top - bottom;
    return difference >= WEIGHT_DIFFERENCE_MM;
}


void navigator_init()
{
    navigator_stop();
}


static int clearanceValue(int distance)
{
    // 0 currently means nothing useful was detected,
    // so treat it as far away for choosing the clearer side.
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}

static bool obstacleCloserThan(int distance,int threshold)
{
    return (distance > 0 && distance < threshold);
}

enum CentreObjectType
{
    CENTRE_UNKNOWN,
    CENTRE_WEIGHT,
    CENTRE_WALL
};


// From both wall tests, the middle weight ToF reads about
// 150 mm farther than the average of the two inner nav ToFs
// when looking at the same wall.
static const int CENTRE_WALL_OFFSET_MM = 150;


// How close to the predicted wall distance counts as wall.
static const int CENTRE_WALL_TOLERANCE_MM = 60;


// Middle must protrude this much in front of the predicted
// wall surface before we trust it as a separate object.
static const int CENTRE_WEIGHT_PROTRUSION_MM = 60;


// If both nav sensors see nothing, only allow centre-only
// detection once reasonably close. This prevents a distant
// wall entering the middle sensor's range first.
static const int CENTRE_ONLY_DETECT_MM = 400;


// Both nav sensors should see roughly the same broad surface.

static CentreObjectType classifyCentreObject(
    int middle,
    int innerLeft,
    int innerRight)
{
    // No centre object.
    if (middle <= 0 ||
        middle > WEIGHT_DETECT_DISTANCE_MM)
    {
        return CENTRE_UNKNOWN;
    }


    // -1 means an unusable ToF reading.
    //
    // VERY IMPORTANT:
    // do not assume "not wall" when the supporting
    // sensors have bad data.
    if (innerLeft < 0 ||
        innerRight < 0)
    {
        return CENTRE_UNKNOWN;
    }


    // --------------------------------------------------------
    // Both nav sensors see no obstacle.
    //
    // A reasonably close object seen ONLY by the narrow
    // middle sensor is a good weight candidate.
    // --------------------------------------------------------

    if (innerLeft == 0 &&
        innerRight == 0)
    {
        if (middle <=
            CENTRE_ONLY_DETECT_MM)
        {
            return CENTRE_WEIGHT;
        }

        return CENTRE_UNKNOWN;
    }


    // One sees something and the other sees nothing.
    // Geometry is ambiguous: wait for better measurements.
    if (innerLeft == 0 ||
        innerRight == 0)
    {
        return CENTRE_UNKNOWN;
    }


    // --------------------------------------------------------
    // Both inner navigation sensors have valid ranges.
    // --------------------------------------------------------

    int innerAverage =
        (innerLeft + innerRight) / 2;

    int innerDifference =
        abs(innerLeft - innerRight);


    int expectedMiddleForWall =
        innerAverage +
        CENTRE_WALL_OFFSET_MM;


    // Positive means the middle sensor sees something CLOSER
    // than where the wall should be.
    int protrusion =
        expectedMiddleForWall -
        middle;


    // --------------------------------------------------------
    // WALL
    // --------------------------------------------------------

    if (innerDifference <=
            INNER_WALL_AGREEMENT_MM &&
        abs(protrusion) <=
            CENTRE_WALL_TOLERANCE_MM)
    {
        return CENTRE_WALL;
    }


    // --------------------------------------------------------
    // WEIGHT / discrete object
    //
    // Middle sees something substantially in front of the
    // surface suggested by the two nav sensors.
    // --------------------------------------------------------

    if (protrusion >=
        CENTRE_WEIGHT_PROTRUSION_MM)
    {
        return CENTRE_WEIGHT;
    }


    // Measurements don't currently prove either case.
    return CENTRE_UNKNOWN;
}


//for now it just has the ability to  look for weights and riase flags
static void detect_weights_exe()
{
    unsigned long now = millis();


    // --------------------------------------------------------
    // Temporary lockout after rejecting a dummy / reversing.
    // --------------------------------------------------------
    if ((int32_t)(now - weightDetectionBlockedUntil) < 0)
    {
        return;
    }


    // Track which actual ToF samples have already been used.
    static uint32_t lastLeftTopSample = 0;
    static uint32_t lastLeftBottomSample = 0;

    static uint32_t lastRightTopSample = 0;
    static uint32_t lastRightBottomSample = 0;

    static uint32_t lastMiddleDetectionSample = 0;


    // Time of last positive piece of evidence.
    static unsigned long leftEvidenceAt = 0;
    static unsigned long rightEvidenceAt = 0;
    static unsigned long middleEvidenceAt = 0;


    // --------------------------------------------------------
    // Read current distances
    // --------------------------------------------------------

    int leftTop =
        tof_get_weight_left_top();

    int leftBottom =
        tof_get_weight_left_bottom();

    int rightTop =
        tof_get_weight_right_top();

    int rightBottom =
        tof_get_weight_right_bottom();

    int middle =
        tof_get_weight_middle();


    int leftNav =
        tof_get_nav_outer_left();

    int rightNav =
        tof_get_nav_outer_right();

    int innerLeft =
        tof_get_nav_inner_left();

    int innerRight =
        tof_get_nav_inner_right();


    // --------------------------------------------------------
    // Read sample numbers
    // --------------------------------------------------------

    uint32_t leftTopSample =
        tof_get_sample_number(
            WEIGHT_LEFT_TOP_SENSOR
        );

    uint32_t leftBottomSample =
        tof_get_sample_number(
            WEIGHT_LEFT_BOTTOM_SENSOR
        );

    uint32_t rightTopSample =
        tof_get_sample_number(
            WEIGHT_RIGHT_TOP_SENSOR
        );

    uint32_t rightBottomSample =
        tof_get_sample_number(
            WEIGHT_RIGHT_BOTTOM_SENSOR
        );

    uint32_t middleSample =
        tof_get_sample_number(
            WEIGHT_MIDDLE_SENSOR
        );


    // --------------------------------------------------------
    // Expire OLD evidence.
    //
    // Important:
    // -1 means unusable measurement.
    // It should not instantly erase a good previous reading,
    // but neither should one old hit survive forever.
    // --------------------------------------------------------

    if (leftDetectionCount > 0 &&
        now - leftEvidenceAt >
            WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        leftDetectionCount = 0;
    }

    if (rightDetectionCount > 0 &&
        now - rightEvidenceAt >
            WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        rightDetectionCount = 0;
    }

    if (middleDetectionCount > 0 &&
        now - middleEvidenceAt >
            WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        middleDetectionCount = 0;
    }


    // --------------------------------------------------------
    // LEFT PAIR
    // --------------------------------------------------------

    if (leftTopSample != lastLeftTopSample &&
        leftBottomSample != lastLeftBottomSample)
    {
        lastLeftTopSample =
            leftTopSample;

        lastLeftBottomSample =
            leftBottomSample;


        // -1 means this sample is unusable.
        // Do NOT destroy previous evidence because of it.
        if (leftTop < 0 ||
            leftBottom < 0)
        {
            // No decision this sample.
        }
        else
        {
            int difference = 0;

            if (weightPairDetected(
                    leftTop,
                    leftBottom,
                    leftNav,
                    difference))
            {
                leftDetectionCount++;
                leftEvidenceAt = now;
            }
            else
            {
                // This was a usable measurement which
                // actively failed the weight test.
                leftDetectionCount = 0;
            }
        }
    }


    // --------------------------------------------------------
    // RIGHT PAIR
    // --------------------------------------------------------

    if (rightTopSample != lastRightTopSample &&
        rightBottomSample != lastRightBottomSample)
    {
        lastRightTopSample =
            rightTopSample;

        lastRightBottomSample =
            rightBottomSample;


        if (rightTop < 0 ||
            rightBottom < 0)
        {
            // Unusable sample: no decision.
        }
        else
        {
            int difference = 0;

            if (weightPairDetected(
                    rightTop,
                    rightBottom,
                    rightNav,
                    difference))
            {
                rightDetectionCount++;
                rightEvidenceAt = now;
            }
            else
            {
                rightDetectionCount = 0;
            }
        }
    }


    // --------------------------------------------------------
    // CENTRE SENSOR
    // --------------------------------------------------------

    if (middleSample !=
    lastMiddleDetectionSample)
    {
        lastMiddleDetectionSample =
            middleSample;


        CentreObjectType centreType =
            classifyCentreObject(
                middle,
                innerLeft,
                innerRight
            );


        if (centreType ==
            CENTRE_WEIGHT)
        {
            middleDetectionCount++;
            middleEvidenceAt = now;


            debugNav.print(
                "CENTRE WEIGHT EVIDENCE "
            );
            debugNav.print(
                middleDetectionCount
            );
            debugNav.print("/");
            debugNav.print(
                MIDDLE_DETECTION_COUNT_REQUIRED
            );

            debugNav.print(
                " IL="
            );
            debugNav.print(innerLeft);

            debugNav.print(
                " IR="
            );
            debugNav.print(innerRight);

            debugNav.print(
                " MID="
            );
            debugNav.println(middle);
        }

        else if (centreType ==
                CENTRE_WALL)
        {
            // A confirmed wall must completely clear any
            // accumulated centre-weight evidence.
            middleDetectionCount = 0;


            static unsigned long
                lastCentreWallDebug = 0;


            if (millis() -
                    lastCentreWallDebug >=
                250)
            {
                lastCentreWallDebug =
                    millis();


                int innerAverage =
                    (innerLeft +
                    innerRight) / 2;

                int expectedMiddle =
                    innerAverage +
                    CENTRE_WALL_OFFSET_MM;


                debugNav.print(
                    "CENTRE REJECT WALL: IL="
                );
                debugNav.print(innerLeft);

                debugNav.print(
                    " IR="
                );
                debugNav.print(innerRight);

                debugNav.print(
                    " MID="
                );
                debugNav.print(middle);

                debugNav.print(
                    " EXPECT="
                );
                debugNav.println(
                    expectedMiddle
                );
            }
        }

        else
        {
            // UNKNOWN IS NOT A WEIGHT.
            //
            // This is the critical difference from the previous
            // implementation. Require two consecutive samples
            // whose geometry actually supports a weight.
            middleDetectionCount = 0;
        }
    }


    // --------------------------------------------------------
    // Decide which source has actually identified a target.
    //
    // Centre gets priority because if it can see the object
    // directly ahead, no initial side-direction assumption
    // is necessary.
    // --------------------------------------------------------

    if (middleDetectionCount >=
        MIDDLE_DETECTION_COUNT_REQUIRED)
    {
        weightTargetSide =
            TARGET_CENTRE;

        debugNav.print(
            "WEIGHT CANDIDATE CENTRE MID="
        );
        debugNav.println(middle);
    }

    else if (leftDetectionCount >=
             SIDE_DETECTION_COUNT_REQUIRED)
    {
        weightTargetSide =
            TARGET_LEFT;

        debugNav.println(
            "WEIGHT CANDIDATE LEFT"
        );

        debugNav.print("LT=");
        debugNav.print(leftTop);

        debugNav.print(" LB=");
        debugNav.print(leftBottom);

        debugNav.print(" RT=");
        debugNav.print(rightTop);

        debugNav.print(" RB=");
        debugNav.println(rightBottom);
    }

    else if (rightDetectionCount >=
             SIDE_DETECTION_COUNT_REQUIRED)
    {
        weightTargetSide =
            TARGET_RIGHT;

        debugNav.println(
            "WEIGHT CANDIDATE RIGHT"
        );

        debugNav.print("LT=");
        debugNav.print(leftTop);

        debugNav.print(" LB=");
        debugNav.print(leftBottom);

        debugNav.print(" RT=");
        debugNav.print(rightTop);

        debugNav.print(" RB=");
        debugNav.println(rightBottom);
    }

    else
    {
        return;
    }


    // --------------------------------------------------------
    // Confirm target and hand over to pursuit
    // --------------------------------------------------------

    pursuitState =
        PURSUIT_START;

    leftDetectionCount = 0;
    rightDetectionCount = 0;
    middleDetectionCount = 0;

    motor_control_stop();

    setStateFlag(
        &STATE_FLAGS.target_identified
    );
}

void navigator_stop()
{
    navigatorEnabled = false;
    motor_control_stop();

    turnWatchActive = false;
    roamingState = ROAM_START;
    leftDetectionCount = 0;
    rightDetectionCount = 0;
    middleDetectionCount = 0;
    if (getNavState() == ROAMING) {
        resetStateFlag(&STATE_FLAGS.target_identified);
    }

    if (getNavState() == PURSUIT) {
        setStateFlag(&STATE_FLAGS.target_lost);
    }

    if (getNavState() == REVERSING) {
        setStateFlag(&STATE_FLAGS.reverse_complete);
    }
}

bool navigator_start(bool enablePickup)
{
    if (getCollectState() != IDLE ||
        (getNavState() != ROAMING && getNavState() != STATIONARY)) {
        // debugNav.println("Navigator: start from ROAMING or STATIONARY");
        return false;
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) return false;
    flatPitchReference =
    imu_get_pitch();

    pitchExceededAt = 0;

    reverseTriggeredByPitch = false;

    rampDetectionBlockedUntil = 0;


    debugNav.print(
        "Navigator: flat pitch reference = "
    );
    debugNav.println(
        flatPitchReference
    );
    navigator_stop();

    navigatorEnabled = true;
    roamingPickupEnabled = enablePickup;
    lastNavState = getNavState();

    navigatorStateStarted = millis();
    lastWeightCheckAt = millis();
    roamCommandedPower = 0;
    weightTargetSide = TARGET_NONE;

    // // debugNav.println(enablePickup ? "Navigator: roaming + pickup" : "Navigator: roaming only");
    return true;
}

static void roaming_start_turn(int leftClearance, int rightClearance)
{
    if (abs(leftClearance - rightClearance) > 80) {
        roamTurnDirection = leftClearance > rightClearance ? -1 : 1;
    }

    motor_control_stop();
    motor_control_turn_relative(roamTurnDirection * ROAM_AVOID_TURN_DEG);

    roamingState = ROAM_TURNING;
    roamCommandedPower = 0;
    leftDetectionCount = rightDetectionCount = 0;

    debugNav.println(roamTurnDirection < 0 ? "Roaming: turn LEFT" : "Roaming: turn RIGHT");
}

static void roaming_drive(int power)
{
    if (motor_control_is_driving() && roamCommandedPower == power) return;

    motor_control_drive_heading(roamHeading, power);
    roamCommandedPower = power;
}



static void roaming_exe()
{
     if (roamingPickupEnabled &&
        millis() - navigatorStateStarted >= 1000 &&
        millis() - lastWeightCheckAt >= 100)
    {
        lastWeightCheckAt = millis();

        detect_weights_exe();

        if (STATE_FLAGS.target_identified)
        {
            return;
        }
    }


    if (roamingState == ROAM_TURNING)
    {
        if (motor_control_is_turning()) return;

        roamCheckStartedAt = millis();
        roamingState = ROAM_CHECKING;
        return;
    }

    if (roamingState == ROAM_TURNING)
    {
        if (motor_control_is_turning()) return;

        roamCheckStartedAt = millis();
        roamingState = ROAM_CHECKING;
        return;
    }

    int outerLeft = tof_get_nav_outer_left();
    int innerLeft = tof_get_nav_inner_left();
    int innerRight = tof_get_nav_inner_right();
    int outerRight = tof_get_nav_outer_right();

    static unsigned long navTofInvalidStarted = 0;

    /*bool tofUnavailable =
        outerLeft < 0 ||
        innerLeft < 0 ||
        innerRight < 0 ||
        outerRight < 0;

    if (tofUnavailable)
    {
        if (navTofInvalidStarted == 0)
        {
            navTofInvalidStarted = millis();
        }

        motor_control_stop();

        if (millis() - navTofInvalidStarted > 500)
        {
            navigator_stop();
            // debugNav.println("Roaming stopped: navigation ToF unavailable");
        }

        return;
    }

    navTofInvalidStarted = 0;*/

    bool weakReturn = outerLeft == 0 || innerLeft == 0 || innerRight == 0 || outerRight == 0;

    outerLeft = clearanceValue(outerLeft);
    innerLeft = clearanceValue(innerLeft);
    innerRight = clearanceValue(innerRight);
    outerRight = clearanceValue(outerRight);

    int front = min(innerLeft, innerRight);
    int leftClearance = min(outerLeft, innerLeft);
    int rightClearance = min(outerRight, innerRight);

    if (front <= ROAM_CRITICAL_MM) {
        // navigator_stop();
        motor_control_stop();
        debugNav.println("Roaming stopped: obstacle critically close");
        return;
    }

    switch (roamingState) {
        case ROAM_START:
            motor_control_stop();
            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;
            break;

        case ROAM_TURNING:
            if (motor_control_is_turning()) return;

            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;
            break;

        case ROAM_CHECKING:
            if (millis() - roamCheckStartedAt < 150) return;

            if (front < ROAM_FRONT_BLOCK_MM + 80 ||
                outerLeft < ROAM_SIDE_BLOCK_MM + 30 ||
                outerRight < ROAM_SIDE_BLOCK_MM + 30) {
                roaming_start_turn(leftClearance, rightClearance);
                return;
            }

            roamHeading = imu_get_heading();
            roamCommandedPower = 0;
            roamingState = ROAM_DRIVING;

            roaming_drive(front < ROAM_SLOW_MM ? ROAM_SLOW_POWER : ROAM_POWER);
            break;

        case ROAM_DRIVING:
            if (front < ROAM_FRONT_BLOCK_MM ||
                outerLeft < ROAM_SIDE_BLOCK_MM ||
                outerRight < ROAM_SIDE_BLOCK_MM) {
                roaming_start_turn(leftClearance, rightClearance);
                return;
            }
            roaming_drive(front < ROAM_SLOW_MM ? ROAM_SLOW_POWER : ROAM_POWER);
            break;
    }
}

static bool readFreshMiddleDistance(int &distance)
{
    uint32_t sample =
        tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);

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

    // Ignore whatever middle reading already exists.
    // We want the next fresh sample.
    lastMiddleSample =
        tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);
}


static bool commandNextPursuitScan()
{
    // 0,1 -> 12 degrees
    // 2,3 -> 24 degrees
    // 4,5 -> 36 degrees
    // 6,7 -> 48 degrees
    int level =
        (pursuitScanIndex / 2) + 1;

    float magnitude =
        level * PURSUIT_SCAN_STEP_DEG;

    if (magnitude > PURSUIT_SCAN_MAX_DEG)
    {
        return false;
    }


    // Search the side which originally detected the weight first,
    // then the equivalent angle on the opposite side.
    int direction = (pursuitScanIndex % 2 == 0) ? 1: -1;


    float offset =
        direction * magnitude;

    pursuitScanIndex++;


    debugNav.print("Pursuit: scanning offset ");
    debugNav.print(offset);
    debugNav.println(" deg");


    // motor_control_turn_to() already handles heading wraparound.
    motor_control_turn_to(
        pursuitScanOriginHeading + offset
    );

    return true;
}

static void pursuit_exe()
{
    switch (pursuitState)
    {
        case PURSUIT_START:
        {
            motor_control_stop();

            // Open funnel before attempting to line up.
            smartservo_arms_open();


            pursuitEntryHeading =
                imu_get_heading();


            if (weightTargetSide == TARGET_LEFT)
            {
                pursuitPreferredDirection = -1;
                debugNav.println(
                    "Pursuit: target came from LEFT"
                );
            }
            else if (weightTargetSide == TARGET_RIGHT)
            {
                pursuitPreferredDirection = 1;
                debugNav.println(
                    "Pursuit: target came from RIGHT"
                );
            }
            else if (weightTargetSide == TARGET_CENTRE)
            {
                debugNav.println(
                    "Pursuit: target detected directly ahead"
                );
            }
            else
            {
                debugNav.println(
                    "Pursuit: started without target side"
                );

                setStateFlag(
                    &STATE_FLAGS.target_lost
                );

                pursuitState =
                    PURSUIT_FINISHED;

                return;
            }


            debugNav.print(
                "Pursuit: search origin heading "
            );
            debugNav.println(
                pursuitEntryHeading
            );


            // Do NOT immediately perform a fixed side turn.
            // First give the centre sensor a chance to see the weight.
            resetPursuitScan();

            pursuitState =
                PURSUIT_ACQUIRING;

            break;
        }


        case PURSUIT_TURNING:
        {
            int centreDistance;

            if (readFreshMiddleDistance(centreDistance))
            {
                if (centreDistance > 0 &&
                    centreDistance <= WEIGHT_DETECT_DISTANCE_MM)
                {
                    debugNav.print(
                        "Pursuit: centre found during scan at "
                    );
                    debugNav.print(centreDistance);
                    debugNav.println(" mm");

                    motor_control_stop();

                    // Check again once stationary before committing.
                    pursuitState = PURSUIT_ACQUIRING;
                    return;
                }
            }

            if (motor_control_is_turning())
            {
                return;
            }

            motor_control_stop();

            debugNav.println(
                "Pursuit: scan turn complete"
            );

            lastMiddleSample =
                tof_get_sample_number(
                    WEIGHT_MIDDLE_SENSOR
                );

            pursuitState = PURSUIT_ACQUIRING;
            break;
        }


        case PURSUIT_ACQUIRING:
        {
            int centreDistance;


            // Only make one decision per actual ToF measurement.
            if (!readFreshMiddleDistance(
                    centreDistance))
            {
                return;
            }


            // -------------------------------
            // Centre has actually found it
            // -------------------------------
            if (centreDistance > 0 &&
                centreDistance <=
                    WEIGHT_DETECT_DISTANCE_MM)
            {
                debugNav.print(
                    "Pursuit: centre acquired weight at "
                );
                debugNav.print(
                    centreDistance
                );
                debugNav.println(
                    " mm"
                );


                middleLostCount = 0;

                weightApproachHeading =
                    imu_get_heading();

                weightApproachSlowed =
                    false;


                pursuitState =
                    PURSUIT_APPROACHING;

                return;
            }


            // -------------------------------
            // Centre still cannot see it
            // -------------------------------
            if (!commandNextPursuitScan())
            {
                motor_control_stop();

                debugNav.println(
                    "Pursuit: scan exhausted - target lost"
                );


                setStateFlag(
                    &STATE_FLAGS.target_lost
                );

                pursuitState =
                    PURSUIT_FINISHED;

                return;
            }


            pursuitState =
                PURSUIT_TURNING;

            break;
        }


        case PURSUIT_APPROACHING:
        {
            int centreDistance;


            // Again, only react to genuinely new middle-ToF samples.
            if (!readFreshMiddleDistance(
                    centreDistance))
            {
                return;
            }


            // -------------------------------
            // Lost target
            // -------------------------------
            if (centreDistance <= 0 || centreDistance > WEIGHT_DETECT_DISTANCE_MM)
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

                    debugNav.println(
                        "Pursuit: centre lost weight - reacquiring"
                    );

                    resetPursuitScan();
                    pursuitState = PURSUIT_ACQUIRING;
                }

                return;
            }


            middleLostCount = 0;


            // -------------------------------
            // Weight has reached funnel
            // -------------------------------
            if (centreDistance <=
                WEIGHT_STOP_DISTANCE_MM)
            {
                motor_control_stop();


                debugNav.print(
                    "Pursuit: weight reached entrance at "
                );
                debugNav.print(
                    centreDistance
                );
                debugNav.println(
                    " mm"
                );


                // Secure weight before SORTING takes control.
                smartservo_arms_close();


                pursuitSecureStartedAt =
                    millis();

                pursuitState =
                    PURSUIT_SECURING;

                return;
            }


            // -------------------------------
            // Slow approach
            // -------------------------------
            if (centreDistance <=
                WEIGHT_SLOW_DISTANCE_MM)
            {
                if (!weightApproachSlowed ||
                    !motor_control_is_driving())
                {
                    motor_control_drive_heading(
                        weightApproachHeading,
                        WEIGHT_SLOW_POWER
                    );


                    weightApproachSlowed =
                        true;


                    debugNav.println(
                        "Pursuit: slowing approach"
                    );
                }

                return;
            }


            // -------------------------------
            // Normal approach
            // -------------------------------
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


            if (millis() -
                    pursuitSecureStartedAt <
                ARM_SECURE_WAIT_MS)
            {
                return;
            }


            debugNav.println(
                "Pursuit: weight secured"
            );


            pursuitState =
                PURSUIT_FINISHED;


            setStateFlag(
                &STATE_FLAGS.weight_in_entrance
            );

            break;
        }


        case PURSUIT_FINISHED:
        {
            // State machine takes over:
            // PURSUIT -> SORTING
            // or
            // PURSUIT -> ROAMING after target_lost.
            break;
        }
    }
}
static int reverseSideClearance(
    int outer,
    int inner)
{
    // -1 = unusable reading.
    // If both are unusable, tell caller we don't know.
    if (outer < 0 && inner < 0)
    {
        return -1;
    }


    // 0 means no obstacle return -> lots of clearance.
    int outerClear =
        outer < 0
            ? -1
            : clearanceValue(outer);

    int innerClear =
        inner < 0
            ? -1
            : clearanceValue(inner);


    if (outerClear < 0)
    {
        return innerClear;
    }

    if (innerClear < 0)
    {
        return outerClear;
    }


    return min(
        outerClear,
        innerClear
    );
}


static void reversing_exe()
{
    switch (reversingState)
    {
        case REVERSE_START:
        {
            motor_control_stop();

            if (reverseTriggeredByPitch)
            {
                debugNav.println(
                    "Reverse: ramp/wall escape - backing away"
                );
            }
            else
            {
                debugNav.println(
                    "Reverse: backing away"
                );
            }


            motor_control_reverse(
                REVERSE_POWER
            );

            reverseStartedAt =
                millis();

            reversingState =
                REVERSE_BACKING;

            break;
        }


        case REVERSE_BACKING:
        {
            unsigned long elapsed =
                millis() -
                reverseStartedAt;


            // ------------------------------------------------
            // Ramp/wall escape:
            // keep reversing until reasonably flat again,
            // with minimum and maximum limits.
            // ------------------------------------------------
            if (reverseTriggeredByPitch)
            {
                float pitchError =
                    fabsf(
                        imu_get_pitch()
                        - flatPitchReference
                    );


                if (elapsed <
                    RAMP_MIN_REVERSE_MS)
                {
                    return;
                }


                if (pitchError >
                        RAMP_RELEASE_DEG &&
                    elapsed <
                        RAMP_MAX_REVERSE_MS)
                {
                    return;
                }
            }

            // ------------------------------------------------
            // Normal dummy/failed collection reverse:
            // retain existing fixed reverse time.
            // ------------------------------------------------
            else
            {
                if (elapsed <
                    REVERSE_TIME_MS)
                {
                    return;
                }
            }


            motor_control_stop();


            // ------------------------------------------------
            // Find which side appears clearer AFTER backing up.
            // ------------------------------------------------

            int leftClearance =
                reverseSideClearance(
                    tof_get_nav_outer_left(),
                    tof_get_nav_inner_left()
                );

            int rightClearance =
                reverseSideClearance(
                    tof_get_nav_outer_right(),
                    tof_get_nav_inner_right()
                );


            int turnDirection = 0;


            // Existing robot sign convention:
            // -1 = left turn
            // +1 = right turn
            if (leftClearance >= 0 &&
                rightClearance >= 0)
            {
                if (leftClearance >
                    rightClearance)
                {
                    turnDirection = -1;

                    debugNav.println(
                        "Reverse: escape turn LEFT"
                    );
                }
                else
                {
                    turnDirection = 1;

                    debugNav.println(
                        "Reverse: escape turn RIGHT"
                    );
                }
            }

            else if (leftClearance >= 0)
            {
                turnDirection = -1;

                debugNav.println(
                    "Reverse: only LEFT clearance known"
                );
            }

            else if (rightClearance >= 0)
            {
                turnDirection = 1;

                debugNav.println(
                    "Reverse: only RIGHT clearance known"
                );
            }

            else
            {
                // Neither side gave useful data.
                // Alternate fallback direction so repeated
                // escapes cannot always choose the same side.
                static int fallbackDirection = 1;

                turnDirection =
                    fallbackDirection;

                fallbackDirection *= -1;

                debugNav.println(
                    "Reverse: clearance unknown - fallback turn"
                );
            }


            debugNav.print(
                "Reverse: left clearance="
            );
            debugNav.print(
                leftClearance
            );

            debugNav.print(
                " right clearance="
            );
            debugNav.println(
                rightClearance
            );


            motor_control_turn_relative(
                turnDirection *
                REVERSE_ESCAPE_TURN_DEG
            );


            reversingState =
                REVERSE_TURNING;

            break;
        }


        case REVERSE_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }


            motor_control_stop();

            debugNav.println(
                "Reverse: escape turn complete"
            );


            reversingState =
                REVERSE_FINISHED;

            break;
        }


        case REVERSE_FINISHED:
        {
            debugNav.println(
                "Reverse: manoeuvre complete"
            );


            // Don't immediately detect the exact same rejected
            // dummy again while leaving it.
            weightDetectionBlockedUntil =
                millis() +
                WEIGHT_RETRIGGER_BLOCK_MS;


            // Don't immediately retrigger pitch either.
            rampDetectionBlockedUntil =
                millis() + 1500;


            leftDetectionCount = 0;
            rightDetectionCount = 0;
            middleDetectionCount = 0;

            weightTargetSide =
                TARGET_NONE;

            reverseTriggeredByPitch =
                false;

            reversingState =
                REVERSE_START;


            setStateFlag(
                &STATE_FLAGS.reverse_complete
            );

            break;
        }
    }
}



void frontier_targetting(){
    int frontier_x = get_frontier_x();
    int frontier_y = get_frontier_y();

    
}

static int homeDockHeading = 0;

static void homing_exe()
{
    // Colour sensor has final authority.
    if (STATE_FLAGS.home_reached && homingState != HOMING_DOCKING && homingState != HOMING_DOCK_TURNING)
    {
        motor_control_stop();

        homeDockStart = millis();
        homeDockHeading = imu_get_heading();

        homingState = HOMING_DOCKING;

        motor_control_drive_heading(
            homeDockHeading,
            HOME_DOCK_POWER
        );

        debugNav.println("Home detected - docking");

        return;
    }
     float distanceHome = homeDistance();

    // We are close enough that steering toward the exact (300,300)
    // coordinate is no longer useful.
    if (distanceHome <= 50 && homingState != HOMING_DOCKING && homingState != HOMING_DOCK_TURNING)
    {
        motor_control_stop();

        debugNav.print(
            "HOMING: inside home arrival zone, distance = "
        );
        debugNav.println(distanceHome);

        return;
    }

    int outerLeft =
        clearanceValue(
            tof_get_nav_outer_left()
        );

    int innerLeft =
        clearanceValue(
            tof_get_nav_inner_left()
        );

    int innerRight =
        clearanceValue(
            tof_get_nav_inner_right()
        );

    int outerRight =
        clearanceValue(
            tof_get_nav_outer_right()
        );

    int front =
        min(innerLeft, innerRight);

    int leftClearance =
        min(outerLeft, innerLeft);

    int rightClearance =
        min(outerRight, innerRight);


    switch (homingState)
    {
        case HOMING_START:
        {
            motor_control_stop();
            debugNav.println("----- HOMING START -----");

            debugNav.print("Pose X = ");
            debugNav.println(pose_get_x_mm());

            debugNav.print("Pose Y = ");
            debugNav.println(pose_get_y_mm());

            debugNav.print("Pose heading = ");
            debugNav.println(pose_get_heading_deg());

            debugNav.print("IMU heading = ");
            debugNav.println(imu_get_heading());

            float dx = arena_get_home_x_mm() - pose_get_x_mm();
            float dy = arena_get_home_y_mm() - pose_get_y_mm();

            debugNav.print("dx home = ");
            debugNav.println(dx);

            debugNav.print("dy home = ");
            debugNav.println(dy);

            float desiredPoseHeading =
                atan2f(dy, dx) * 180.0f / PI;

            debugNav.print("Desired pose heading = ");
            debugNav.println(desiredPoseHeading);

            float turn = homeHeadingError();

            debugNav.print("Homing relative turn = ");
            debugNav.println(turn);
            debugNav.print(
                "Homing turn toward base: "
            );
            debugNav.println(turn);

            if (fabs(turn) > 5.0f)
            {   
                debugNav.print("Commanding home turn = ");
                debugNav.println(turn);
                motor_control_turn_relative(
                    turn
                );

                homingState =
                    HOMING_TURNING;
            }
            else
            {
                homingState =
                    HOMING_DRIVING;
            }

            break;
        }


        case HOMING_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            lastHomeHeadingUpdate = 0;

            homingState =
                HOMING_DRIVING;

            break;
        }


        case HOMING_DRIVING:
        {
            // Basic obstacle avoidance until D* is ready.
            if (front < HOME_FRONT_BLOCK_MM)
            {
                motor_control_stop();

                float turnDirection;

                if (leftClearance >
                    rightClearance)
                {
                    turnDirection =
                        -HOME_AVOID_TURN_DEG;
                }
                else
                {
                    turnDirection =
                        HOME_AVOID_TURN_DEG;
                }

                debugNav.println(
                    "Homing: obstacle avoidance"
                );

                motor_control_turn_relative(
                    turnDirection
                );

                homingState =
                    HOMING_AVOIDING;

                return;
            }


            // Continually correct heading toward the
            // estimated starting location.
            if (
                millis() -
                lastHomeHeadingUpdate
                >= HOME_HEADING_UPDATE_MS
            )
            {
                lastHomeHeadingUpdate =
                    millis();

                float relativeError =
                    homeHeadingError();

                // Convert our relative correction back
                // into an absolute IMU target.
                float targetHeading =
                    imu_get_heading()
                    + relativeError;

                int power =
                    homeDistance()
                        < HOME_SLOW_DISTANCE_MM
                    ? HOME_SLOW_POWER
                    : HOME_POWER;

                motor_control_drive_heading(
                    targetHeading,
                    power
                );
            }

            break;
        }


        case HOMING_AVOIDING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            // After avoiding obstacle, recalculate
            // direction toward home.
            homingState =
                HOMING_START;

            break;
        }

        case HOMING_DOCKING:
        {
            if (millis() - homeDockStart >= HOME_DOCK_TIME_MS)
            {
                motor_control_stop();
                debugNav.println("Docking complete - turning 180");

                motor_control_turn_relative(180.0f);

                homingState = HOMING_DOCK_TURNING;
            }

            break;
        }

        case HOMING_DOCK_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            debugNav.println(
                "Home 180 turn complete"
            );

            resetStateFlag(
                &STATE_FLAGS.home_reached
            );

            setStateFlag(
                &STATE_FLAGS.home_docked
            );

            break;
        }
    }
}

static bool checkForPitchEscape(
    NavState nav)
{
    unsigned long now =
        millis();


    // Only relevant while actually navigating.
    if (nav != ROAMING &&
        nav != PURSUIT &&
        nav != HOMING)
    {
        pitchExceededAt = 0;
        return false;
    }


    // Don't let the little home-base rim interfere with
    // final docking / 180 degree dropoff manoeuvre.
    if (nav == HOMING &&
        (homingState == HOMING_DOCKING ||
         homingState == HOMING_DOCK_TURNING))
    {
        pitchExceededAt = 0;
        return false;
    }


    if ((int32_t)(
            now -
            rampDetectionBlockedUntil
        ) < 0)
    {
        pitchExceededAt = 0;
        return false;
    }


    float pitch =
        imu_get_pitch();

    float pitchDifference =
        fabsf(
            pitch -
            flatPitchReference
        );


    // Robot looks reasonably flat.
    if (pitchDifference <
        RAMP_TRIGGER_DEG)
    {
        pitchExceededAt = 0;
        return false;
    }


    // First abnormal pitch reading.
    if (pitchExceededAt == 0)
    {
        pitchExceededAt = now;
        return false;
    }


    // Require sustained tilt rather than one bump/glitch.
    if (now - pitchExceededAt <
        RAMP_CONFIRM_MS)
    {
        return false;
    }


    // --------------------------------------------------------
    // Genuine ramp / wall-climb event
    // --------------------------------------------------------

    motor_control_stop();

    debugNav.print(
        "PITCH ESCAPE: pitch="
    );
    debugNav.print(pitch);

    debugNav.print(
        " reference="
    );
    debugNav.print(
        flatPitchReference
    );

    debugNav.print(
        " difference="
    );
    debugNav.println(
        pitchDifference
    );


    reverseTriggeredByPitch =
        true;

    pitchExceededAt = 0;


    // Prevent another pitch trigger while we're already
    // transitioning into reverse.
    rampDetectionBlockedUntil =
        now + 3000;


    // If pursuit drove us onto something, abandon that target
    // once the reverse manoeuvre is over.
    if (nav == PURSUIT)
    {
        setStateFlag(
            &STATE_FLAGS.target_lost
        );
    }


    setStateFlag(
        &STATE_FLAGS.reverse_triggered
    );


    return true;
}


void navigator_exe()
{
    if (!navigatorEnabled) return;

    NavState nav = getNavState();

    if (nav != lastNavState) {
        motor_control_stop();

        lastNavState = nav;
        navigatorStateStarted = millis();
        turnWatchActive = false;

        leftDetectionCount = rightDetectionCount = middleLostCount = 0;

        if (nav == ROAMING) {
            roamingState = ROAM_START;
            weightTargetSide = TARGET_NONE;
            roamCommandedPower = 0;
        }

        if (nav == PURSUIT) {
            pursuitState = PURSUIT_START;
            weightApproachSlowed = false;
        }

        if (nav == REVERSING) {
            reversingState = REVERSE_START;
        }
        if (nav == HOMING)
        {
            homingState = HOMING_START;
            lastHomeHeadingUpdate = 0;

            debugNav.println("Navigator: HOMING started");
        }
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) {
        navigator_stop();
        // debugNav.println("Navigator stopped: IMU unavailable");
        return;
    }

    if (checkForPitchEscape(nav))
    {
        return;
    }

    if (!motor_control_is_turning()) turnWatchActive = false;

    if (turnWatchActive && millis() - turnStartedAt >= NAV_TURN_TIMEOUT_MS) {
        motor_control_stop();

        turnWatchActive = false;

        debugNav.println(
            "Navigation turn timeout - recovering"
        );

        if (nav == PURSUIT)
        {
            setStateFlag(
                &STATE_FLAGS.target_lost
            );
        }
        else if (nav == ROAMING)
        {
            roamingState =
                ROAM_START;
        }
        else if (nav == HOMING)
        {
            homingState =
                HOMING_START;
        }
        return;
    }

    if (nav == PURSUIT &&
    pursuitState != PURSUIT_SECURING &&
    pursuitState != PURSUIT_FINISHED)
    {
        if (millis() - navigatorStateStarted >= PURSUIT_TIMEOUT_MS)
        {
            motor_control_stop();

            debugNav.println("Pursuit: timeout");

            setStateFlag(&STATE_FLAGS.target_lost);

            return;
        }
    }

    switch (nav) {
        case ROAMING:
            roaming_exe();
            break;

        case PURSUIT:
            pursuit_exe();
            break;

        case REVERSING:
            reversing_exe();
            break;

        case HOMING:
            homing_exe();
            // debugNav.println("Navigator stopped: homing not implemented");
            break;

        default:
            break;
    }

    if (motor_control_is_turning() && !turnWatchActive) {
        turnStartedAt = millis();
        turnWatchActive = true;
    }
}

