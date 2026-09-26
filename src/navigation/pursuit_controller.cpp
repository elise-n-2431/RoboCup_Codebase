#include "pursuit_controller.h"

#include <Arduino.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "outputs/smart_servo.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "debug_print.h"


// Pursuit tuning
static const int WEIGHT_DETECT_DISTANCE_MM = 650;
static const int WEIGHT_MIDDLE_SENSOR = 8;

static const int MIDDLE_LOST_COUNT_REQUIRED = 3;

static const float PURSUIT_SCAN_STEP_DEG = 15.0f;
static const float PURSUIT_SCAN_MAX_DEG = 60.0f;

static const int WEIGHT_STOP_DISTANCE_MM = 90;
static const int WEIGHT_SLOW_DISTANCE_MM = 200;

static const int WEIGHT_APPROACH_POWER = 420;
static const int WEIGHT_SLOW_POWER = 300;
static const int PURSUIT_WALL_ABORT_MM = 25;

static const unsigned long ARM_SECURE_WAIT_MS = 1000;
static const unsigned long PURSUIT_TIMEOUT_MS = 10000;


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
static WeightTargetSide weightTargetSide = TARGET_NONE;

static int middleLostCount = 0;

static float weightApproachHeading = 0.0f;
static bool weightApproachSlowed = false;

static float pursuitEntryHeading = 0.0f;
static float pursuitScanOriginHeading = 0.0f;

static int pursuitScanIndex = 0;
static int pursuitPreferredDirection = 1;

static uint32_t lastMiddleSample = 0;

static unsigned long pursuitSecureStartedAt = 0;
static unsigned long pursuitStartedAt = 0;

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
    int stepsPerSide =
        (int)(
            PURSUIT_SCAN_MAX_DEG /
            PURSUIT_SCAN_STEP_DEG
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
        PURSUIT_SCAN_STEP_DEG;

    pursuitScanIndex++;

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
    pursuitPreferredDirection = 1;
    pursuitState = PURSUIT_START;

    middleLostCount = 0;
    weightApproachSlowed = false;

    pursuitScanIndex = 0;
    pursuitSecureStartedAt = 0;

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


static int getPursuitFrontClearance()
{
    int innerLeft =
        pursuitClearanceValue(
            tof_get_nav_inner_left()
        );

    int innerRight =
        pursuitClearanceValue(
            tof_get_nav_inner_right()
        );

    return min(
        innerLeft,
        innerRight
    );
}

void pursuit_update()
{   
    if (pursuitStartedAt != 0 &&
        pursuitState != PURSUIT_SECURING &&
        pursuitState != PURSUIT_FINISHED &&
        millis() - pursuitStartedAt >
            PURSUIT_TIMEOUT_MS)
    {
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

            // Give the centre sensor a chance to find the weight
            // before performing any fixed side turn.
            resetPursuitScan();

            pursuitState = PURSUIT_ACQUIRING;
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
                    debugNav.print("Pursuit: centre found during scan at ");
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
            debugNav.println("Pursuit: scan turn complete");

            lastMiddleSample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);

            pursuitState = PURSUIT_ACQUIRING;
            break;
        }

        case PURSUIT_ACQUIRING:
        {
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
                debugNav.print("Pursuit: centre acquired weight at ");
                debugNav.print(centreDistance);
                debugNav.println(" mm");

                middleLostCount = 0;
                weightApproachHeading = imu_get_heading();
                weightApproachSlowed = false;

                pursuitState = PURSUIT_APPROACHING;
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
            int centreDistance;

            // Only react to genuinely new middle-ToF samples.
            if (!readFreshMiddleDistance(centreDistance))
            {
                return;
            }
            int frontClearance =
                getPursuitFrontClearance();

            bool weightAlreadyAtEntrance =
                centreDistance > 0 &&
                centreDistance <=
                    WEIGHT_STOP_DISTANCE_MM;

            if (!weightAlreadyAtEntrance &&
                frontClearance <=
                    PURSUIT_WALL_ABORT_MM)
            {
                motor_control_stop();

                debugNav.print(
                    "NAV_EVENT,"
                );

                debugNav.print(
                    millis()
                );

                debugNav.print(
                    ",PURSUIT_WALL_ABORT,"
                );

                debugNav.print(
                    frontClearance
                );

                debugNav.print(
                    ",MIDDLE,"
                );

                debugNav.println(
                    centreDistance
                );

                setStateFlag(
                    &STATE_FLAGS.target_lost
                );

                pursuitState =
                    PURSUIT_FINISHED;

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

            // Weight has reached the funnel.
            if (centreDistance <= WEIGHT_STOP_DISTANCE_MM)
            {
                motor_control_stop();

                debugNav.print("Pursuit: weight reached entrance at ");
                debugNav.print(centreDistance);
                debugNav.println(" mm");

                // Secure weight before SORTING takes control.
                smartservo_arms_close();

                pursuitSecureStartedAt = millis();
                pursuitState = PURSUIT_SECURING;
                return;
            }

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

            debugNav.println("Pursuit: weight secured");

            pursuitState = PURSUIT_FINISHED;
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

    weightApproachHeading = 0.0f;
    weightApproachSlowed = false;

    pursuitEntryHeading = 0.0f;
    pursuitScanOriginHeading = 0.0f;
    pursuitScanIndex = 0;

    pursuitSecureStartedAt = 0;
    pursuitStartedAt = 0;
}