#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "inputs/tof_expander.h"
#include "state_machine.h"




static const int WEIGHT_DETECT_DISTANCE_MM = 550;

static int middleLostCount = 0;
static const int MIDDLE_LOST_COUNT_REQUIRED = 3;


static const float LEFT_WEIGHT_TURN_DEG  = -25.0f;
static const float RIGHT_WEIGHT_TURN_DEG = 25.0f;
static const int WEIGHT_DIFFERENCE_MM = 100;
static const int WEIGHT_STOP_DISTANCE_MM = 55;
static int DETECTION_COUNT_REQUIRED = 3;

static const int WEIGHT_SLOW_DISTANCE_MM = 150;

static const int WEIGHT_APPROACH_POWER = 320;
static const int WEIGHT_SLOW_POWER = 260;


static int leftDetectionCount = 0;
static int rightDetectionCount = 0;
static float weightApproachHeading = 0.0f;
static bool weightApproachSlowed = false;


//In the roaming state to see which side it thinks the weight is on
enum WeightTargetSide
{
    TARGET_NONE,
    TARGET_LEFT,
    TARGET_RIGHT
};

static WeightTargetSide weightTargetSide = TARGET_NONE;


enum PursuitState
{
    PURSUIT_START,
    PURSUIT_TURNING,
    PURSUIT_ACQUIRING,
    PURSUIT_APPROACHING,
    PURSUIT_FINISHED
};

static PursuitState pursuitState = PURSUIT_START;


// ============================================================
// NAVIGATOR STATE
// ============================================================

static float testHeading = 0.0;

static bool testActive = false;


// If further away than this,
// turn before trying to drive.
const float ALIGN_TOLERANCE = 3.0;


// Normal navigation drive power
const int NAV_DRIVE_POWER = 350;


// ============================================================
// ANGLE HELPERS
// ============================================================

static bool weightPairDetected(
    int top,
    int bottom,
    int &difference
)
{
    difference = 0;

    // Bottom sensor must actually see something
    if (bottom <= 0)
    {
        return false;
    }

    // Don't identify distant objects as weights
    if (bottom > WEIGHT_DETECT_DISTANCE_MM)
    {
        return false;
    }

    // Bottom sees something while top sees nothing:
    // strong evidence of a short object
    if (top <= 0)
    {
        return true;
    }

    // Both sensors see something.
    // A closer bottom reading indicates a short object
    // sticking out from the background.
    difference =
        top - bottom;

    return (
        difference >= WEIGHT_DIFFERENCE_MM
    );
}

static float wrapHeading(float heading)
{
    while (heading >= 360.0)
    {
        heading -= 360.0;
    }


    while (heading < 0.0)
    {
        heading += 360.0;
    }


    return heading;
}


static float headingError(
    float target,
    float current
)
{
    float error =
        target - current;


    while (error > 180.0)
    {
        error -= 360.0;
    }


    while (error < -180.0)
    {
        error += 360.0;
    }


    return error;
}


// ============================================================
// INITIALISE
// ============================================================

void navigator_init()
{
    testHeading = 0.0;

    testActive = false;
}


// ============================================================
// TEMPORARY ABSOLUTE HEADING TEST
// ============================================================

void navigator_setTestHeading(float heading)
{
    testHeading =
        wrapHeading(heading);


    // New navigator command replaces any direct
    // turn/drive controller command.
    if (motor_control_is_active())
    {
        motor_control_stop();
    }


    testActive = true;


    Serial.print(
        "Navigator target heading: "
    );

    Serial.println(
        testHeading
    );


    Serial2.print(
        "Navigator target heading: "
    );

    Serial2.println(
        testHeading
    );
}






//for now it just has the ability to  look for weights and riase flags
static void roaming_exe()
{
    int leftTop =
        tof_get_weight_left_top();

    int leftBottom =
        tof_get_weight_left_bottom();

    int rightTop =
        tof_get_weight_right_top();

    int rightBottom =
        tof_get_weight_right_bottom();


    int leftDifference = 0;
    int rightDifference = 0;


    bool leftDetected = weightPairDetected(leftTop,leftBottom,leftDifference);

    bool rightDetected = weightPairDetected(rightTop,rightBottom,rightDifference);


    if (leftDetected)
    {
        leftDetectionCount++;
    }
    else
    {
        leftDetectionCount = 0;
    }


    if (rightDetected)
    {
        rightDetectionCount++;
    }
    else
    {
        rightDetectionCount = 0;
    }

    static unsigned long lastRoamPrint = 0;

    if (millis() - lastRoamPrint >= 250)
    {
        lastRoamPrint = millis();

        Serial2.print("ROAM | LT=");
        Serial2.print(leftTop);
        Serial2.print(" LB=");
        Serial2.print(leftBottom);
        Serial2.print(" | RT=");
        Serial2.print(rightTop);
        Serial2.print(" RB=");
        Serial2.println(rightBottom);
    }


   

    if (leftDetectionCount >= DETECTION_COUNT_REQUIRED)
    {
        weightTargetSide = TARGET_LEFT;
        pursuitState = PURSUIT_START;

        leftDetectionCount = 0;
        rightDetectionCount = 0;

        Serial.println("Roaming: weight detected LEFT");

        setStateFlag(&STATE_FLAGS.target_identified);
        return;
    }

    

    if (rightDetectionCount >= DETECTION_COUNT_REQUIRED)
    {
        weightTargetSide = TARGET_RIGHT;
        pursuitState = PURSUIT_START;

        leftDetectionCount = 0;
        rightDetectionCount = 0;

        Serial.println("Roaming: weight detected RIGHT");

        setStateFlag(&STATE_FLAGS.target_identified);
        return;
    }

}

// ============================================================
// PURSUIT
// ============================================================
static void pursuit_exe()
{
    switch (pursuitState)
    {
        case PURSUIT_START:
        {
            if (weightTargetSide == TARGET_LEFT)
            {
                Serial.println("Pursuit: turning LEFT toward weight");
                motor_control_turn_relative(LEFT_WEIGHT_TURN_DEG);
                pursuitState = PURSUIT_TURNING;
            }
            else if (weightTargetSide == TARGET_RIGHT)
            {
                Serial.println("Pursuit: turning RIGHT toward weight");
                motor_control_turn_relative(RIGHT_WEIGHT_TURN_DEG);
                pursuitState = PURSUIT_TURNING;
            }
            else
            {
                Serial.println("Pursuit started without target side");
            }

            break;
        }

        case PURSUIT_TURNING:
        {
            if (motor_control_is_turning()) return;

            Serial.println("Pursuit: turn complete");
            pursuitState = PURSUIT_ACQUIRING;

            break;
        }

        case PURSUIT_ACQUIRING:
        {
            int centreDistance = tof_get_weight_middle();

            if (centreDistance > 0 && centreDistance <= WEIGHT_DETECT_DISTANCE_MM)
            {
                Serial.print("Pursuit: centre acquired weight at ");
                Serial.print(centreDistance);
                Serial.println(" mm");

                middleLostCount = 0;
                weightApproachHeading = imu_get_heading();
                weightApproachSlowed = false;

                pursuitState = PURSUIT_APPROACHING;
                return;
            }

            break;
        }

        case PURSUIT_APPROACHING:
        {
            int centreDistance = tof_get_weight_middle();

            // Lost target
            if (centreDistance <= 0)
            {
                middleLostCount++;

                if (middleLostCount >= MIDDLE_LOST_COUNT_REQUIRED)
                {
                    motor_control_stop();

                    Serial.println("Pursuit: centre lost weight");

                    middleLostCount = 0;
                    pursuitState = PURSUIT_ACQUIRING;
                }

                return;
            }

            middleLostCount = 0;

            // Weight has reached the entrance
            if (centreDistance <= WEIGHT_STOP_DISTANCE_MM)
            {
                motor_control_stop();

                Serial.print("Pursuit: weight reached at ");
                Serial.print(centreDistance);
                Serial.println(" mm");

                pursuitState = PURSUIT_FINISHED;
                setStateFlag(&STATE_FLAGS.weight_in_entrance);

                return;
            }

            // Slow down near the weight
            if (centreDistance <= WEIGHT_SLOW_DISTANCE_MM)
            {
                if (!weightApproachSlowed)
                {
                    motor_control_drive_heading(weightApproachHeading, WEIGHT_SLOW_POWER);
                    weightApproachSlowed = true;

                    Serial.println("Pursuit: slowing approach");
                }

                return;
            }

            // Start normal approach
            if (!motor_control_is_driving())
            {
                motor_control_drive_heading(weightApproachHeading, WEIGHT_APPROACH_POWER);
            }

            break;
        }

        case PURSUIT_FINISHED:
        {
            // Wait for top-level state machine to move PURSUIT -> SORTING
            break;
        }
    }
}




void navigator_exe()
{
    switch (getNavState())
    {
        case ROAMING:
            roaming_exe();
            break;

        case PURSUIT:
            pursuit_exe();
            break;

        case HOMING:
            // homing_exe();
            break;

        default:
            break;
    }
}




// ============================================================
// STOP NAVIGATION
// ============================================================

void navigator_stop()
{
    testActive = false;
}


// ============================================================
// STATE
// ============================================================

bool navigator_is_active()
{
    return testActive;
}
