#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"


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