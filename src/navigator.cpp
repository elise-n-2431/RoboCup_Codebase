#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "inputs/tof_expander.h"
#include "unused/collection.h"


enum WeightTestState
{
    WEIGHT_TEST_IDLE,
    WEIGHT_TEST_SEARCHING,
    WEIGHT_TEST_TURNING,
    WEIGHT_TEST_APPROACHING,
    WEIGHT_TEST_COLLECTING,
    WEIGHT_TEST_FINISHED
};

static WeightTestState weightTestState = WEIGHT_TEST_IDLE;

static const int WEIGHT_DETECT_MAX_MM = 600;

static const int APPROACH_SLOW_MM = 300;
static const int APPROACH_STOP_MM = 120;

static const int WEIGHT_DETECT_DISTANCE_MM = 550;

static unsigned long lastWeightDebugPrint = 0;

static const unsigned long
    WEIGHT_DEBUG_PERIOD_MS = 250;

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


    // Bottom sensor must actually see an object
    if (bottom <= 0)
    {
        return (
        bottom <= WEIGHT_DETECT_DISTANCE_MM
    );
    }


    // Top sees nothing, but bottom does.
    // Strong evidence of a short object.
    if (top <= 0)
    {
        return (
        top <= WEIGHT_DETECT_DISTANCE_MM
    );
    }


    // Both sensors see something.
    // Compare their distances.
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


// ============================================================
// NAVIGATOR
// ============================================================

void navigator_exe()
{
    if (!testActive)
    {
        return;
    }


    float currentHeading =
        imu_get_heading();


    float error =
        headingError(
            testHeading,
            currentHeading
        );


    // ========================================================
    // NOT ALIGNED:
    // TURN TO THE DESIRED HEADING
    // ========================================================

    if (fabs(error) > ALIGN_TOLERANCE)
    {
        if (!motor_control_is_turning())
        {
            motor_control_turn_to(
                testHeading
            );
        }
        else
        {
            // Don't restart the controller every loop.
            // Just update its target.
            motor_control_set_target_heading(
                testHeading
            );
        }


        return;
    }


    // ========================================================
    // ALIGNED:
    // DRIVE WHILE HOLDING THE HEADING
    // ========================================================

    if (!motor_control_is_driving())
    {
        motor_control_drive_heading(
            testHeading,
            NAV_DRIVE_POWER
        );
    }
    else
    {
        motor_control_set_target_heading(
            testHeading
        );
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

void navigator_start_weight_test()
{
    leftDetectionCount = 0;
    rightDetectionCount = 0;

    weightTestState =
        WEIGHT_TEST_SEARCHING;


    Serial2.println(
        "Weight pursuit test started"
    );
}

void navigator_stop_weight_test()
{
    motor_control_stop();

    weightTestState =
        WEIGHT_TEST_IDLE;


    Serial2.println(
        "Weight pursuit test stopped"
    );
}
void navigator_update_weight_test()
{
    if (
        weightTestState
        == WEIGHT_TEST_IDLE
    )
    {
        return;
    }


    // ========================================================
    // SEARCHING
    // ========================================================

    if (
        weightTestState
        == WEIGHT_TEST_SEARCHING
    )
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


        
        bool leftDetected =
            weightPairDetected(
                leftTop,
                leftBottom,
                leftDifference
            );


        bool rightDetected =
            weightPairDetected(
                rightTop,
                rightBottom,
                rightDifference
            );


        // ====================================================
        // CONSECUTIVE DETECTION COUNTS
        // ====================================================

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

        unsigned long now =
    millis();


        if (
            now - lastWeightDebugPrint
            >= WEIGHT_DEBUG_PERIOD_MS
        )
        {
            lastWeightDebugPrint =
                now;


            Serial2.print("WEIGHT SEARCH | ");

            Serial2.print("LT=");
            Serial2.print(leftTop);

            Serial2.print(" LB=");
            Serial2.print(leftBottom);

            Serial2.print(" DiffL=");
            Serial2.print(leftDifference);


            Serial2.print(" | RT=");
            Serial2.print(rightTop);

            Serial2.print(" RB=");
            Serial2.print(rightBottom);

            Serial2.print(" DiffR=");
            Serial2.print(rightDifference);


            Serial2.print(" | M=");
            Serial2.println(
                tof_get_weight_middle()
            );
        }


        // ====================================================
        // LEFT DETECTION
        // ====================================================

        if (
            leftDetectionCount
            >= DETECTION_COUNT_REQUIRED
        )
        {
            Serial2.print(
                "Weight detected LEFT. Difference: "
            );

            Serial2.println(
                leftDifference
            );


            leftDetectionCount = 0;
            rightDetectionCount = 0;


            motor_control_turn_relative(
                LEFT_WEIGHT_TURN_DEG
            );


            weightTestState =
                WEIGHT_TEST_TURNING;


            return;
        }


        // ====================================================
        // RIGHT DETECTION
        // ====================================================

        if (
            rightDetectionCount
            >= DETECTION_COUNT_REQUIRED
        )
        {
            Serial2.print(
                "Weight detected RIGHT. Difference: "
            );

            Serial2.println(
                rightDifference
            );


            leftDetectionCount = 0;
            rightDetectionCount = 0;


            motor_control_turn_relative(
                RIGHT_WEIGHT_TURN_DEG
            );


            weightTestState =
                WEIGHT_TEST_TURNING;


            return;
        }


        return;
    }


    // ========================================================
    // TURNING
    // ========================================================

    if (
        weightTestState
        == WEIGHT_TEST_TURNING
    )
    {
        // Let the existing motor controller finish
        // the commanded turn first.
        if (motor_control_is_turning())
        {
            return;
        }


        // Turn has now finished.
        int centreDistance =
            tof_get_weight_middle();
        Serial2.print(
            "TURN FINISHED | Middle = "
        );

        Serial2.println(
            centreDistance
        );


        // ====================================================
        // CENTRE SENSOR ACQUIRED TARGET
        // ====================================================

        if (
            centreDistance > 0
            &&
            centreDistance
                <= WEIGHT_DETECT_DISTANCE_MM
        )
        {
            Serial2.print(
                "Centre acquired weight at "
            );

            Serial2.print(
                centreDistance
            );

            Serial2.println(
                " mm"
            );

            middleLostCount = 0;


            // Save this heading.
            // Do NOT keep resetting it to imu_get_heading()
            // every loop.
            weightApproachHeading =
                imu_get_heading();


            weightApproachSlowed =
                false;


            motor_control_drive_heading(
                weightApproachHeading,
                WEIGHT_APPROACH_POWER
            );


            weightTestState =
                WEIGHT_TEST_APPROACHING;


            return;
        }


        // ====================================================
        // CENTRE DID NOT ACQUIRE TARGET
        // ====================================================

        Serial2.println(
            "Centre did not acquire weight - rechecking side sensors"
        );


        leftDetectionCount = 0;
        rightDetectionCount = 0;


        weightTestState =
            WEIGHT_TEST_SEARCHING;


        return;
    }


    // ========================================================
    // APPROACHING
    // ========================================================

    if (
        weightTestState
        == WEIGHT_TEST_APPROACHING
    )
    {
        int centreDistance =
            tof_get_weight_middle();


        // ====================================================
        // LOST TARGET
        // ====================================================

        if (centreDistance <= 0)
        {
            middleLostCount++;

            if (
                middleLostCount
                >= MIDDLE_LOST_COUNT_REQUIRED
            )
            {
                motor_control_stop();

                Serial2.println(
                    "Centre lost weight - reacquiring"
                );

                leftDetectionCount = 0;
                rightDetectionCount = 0;
                middleLostCount = 0;

                weightTestState =
                    WEIGHT_TEST_SEARCHING;
            }

            return;
        }


        // Got a valid centre reading again
        middleLostCount = 0;


        // ====================================================
        // REACHED WEIGHT
        // ====================================================

        if (
            centreDistance
            <= WEIGHT_STOP_DISTANCE_MM
        )
        {
            motor_control_stop();


            Serial2.print(
                "Weight reached at "
            );

            Serial2.print(
                centreDistance
            );

            Serial2.println(
                " mm"
            );


            collection_start();


            weightTestState =
                WEIGHT_TEST_COLLECTING;


            return;
        }


        // ====================================================
        // SLOW DOWN
        // ====================================================

        if (
            centreDistance
                <= WEIGHT_SLOW_DISTANCE_MM
        )
        {
            // Only change drive mode/power ONCE.
            if (!weightApproachSlowed)
            {
                motor_control_drive_heading(
                    weightApproachHeading,
                    WEIGHT_SLOW_POWER
                );


                weightApproachSlowed =
                    true;


                Serial2.println(
                    "Slowing approach"
                );
            }


            return;
        }


        // ====================================================
        // NORMAL APPROACH
        // ====================================================

        // Motor controller is already driving toward
        // weightApproachHeading.
        //
        // Do not restart motor_control_drive_heading()
        // every navigator loop.

        return;
    }



        if (
            weightTestState
            == WEIGHT_TEST_COLLECTING
        )
        {
            // collection_exe() is running from main loop.
            // Once it returns to IDLE, collection is complete.
            if (!collectionIsActive())
            {
                Serial2.println(
                    "Weight collection complete"
                );

                weightTestState =
                    WEIGHT_TEST_FINISHED;
            }


            return;
        }

    // ========================================================
    // FINISHED
    // ========================================================

    if (
        weightTestState
        == WEIGHT_TEST_FINISHED
    )
    {
        return;
    }
}
