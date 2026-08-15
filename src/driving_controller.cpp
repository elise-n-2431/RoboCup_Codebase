#include "driving_controller.h"

#include <Arduino.h>
#include <math.h>

#include "outputs/DC_motors.h"
#include "inputs/imu.h"



// To be tuned
static float KP = 7.0;
static float KD = 0.0;


// Minimum power for robot to actually rotate
const int MIN_TURN_POWER = 300;

// Keep early testing fairly gentle
const int MAX_TURN_POWER = 450;


// Consider target reached inside this angle
const float ANGLE_TOLERANCE = 2.5;


// Must stay in tolerance this long before finishing
const unsigned long SETTLE_TIME_MS = 150;


// If the robot turns AWAY from the target,
// change this from +1 to -1
const int TURN_SIGN = 1;



static bool controlActive = false; //for testing, wehrther control being used

static float targetHeading = 0.0;
static float currentError = 0.0;
static float previousError = 0.0;

static unsigned long previousTime = 0;
static unsigned long toleranceStart = 0;


//to make sure the angle is always positive for PD math
static float wrapHeading(float heading)
{
    while (heading >= 360.0) {
        heading -= 360.0;
    }

    while (heading < 0.0) {
        heading += 360.0;
    }

    return heading;
}

//also wraps the heading angle error
static float headingError(float target, float current)
{
    float error = target - current;

    while (error > 180.0) {
        error -= 360.0;
    }

    while (error < -180.0) {
        error += 360.0;
    }

    return error;
}



void motor_control_init()
{
    controlActive = false;

    targetHeading = 0.0;
    currentError = 0.0;
    previousError = 0.0;

    previousTime = millis();
    toleranceStart = 0;

    DC_motors_setPower(0, 0);
}


//turn to an angle relative to a starting psoition, starting this with bluetooth
void motor_control_turn_relative(float angle)
{
    float currentHeading = imu_get_heading();

    targetHeading =
        wrapHeading(currentHeading + angle);

    currentError =
        headingError(targetHeading, currentHeading);

    previousError = currentError;

    previousTime = millis();
    toleranceStart = 0;

    controlActive = true;

    //for testing printing to bluetooth as well as serial when plugged into teensy
    Serial.print("Current heading: ");
    Serial.println(currentHeading);

    Serial.print("Relative turn: ");
    Serial.println(angle);

    Serial.print("Target heading: ");
    Serial.println(targetHeading);

    Serial2.print("Current heading: ");
    Serial2.println(currentHeading);

    Serial2.print("Relative turn: ");
    Serial2.println(angle);

    Serial2.print("Target heading: ");
    Serial2.println(targetHeading);
}



void motor_control_turn_to(float heading)
{
    targetHeading = wrapHeading(heading);

    float currentHeading = imu_get_heading();

    currentError = headingError(targetHeading, currentHeading);

    previousError = currentError;

    previousTime = millis();
    toleranceStart = 0;

    controlActive = true;
}



void motor_control_update()
{
    if (!controlActive) {
        return;
    }


    unsigned long currentTime = millis();

    float dt =
        (currentTime - previousTime) / 1000.0;


    if (dt <= 0.0) {
        return;
    }


    float currentHeading = imu_get_heading();


    currentError = headingError(targetHeading, currentHeading);


    //checking if the target has been reached
    if (fabs(currentError) <= ANGLE_TOLERANCE)
    {
        // target reached so for testing turn off the motors
        //has to be wihtin the tolerance for a certain amount of time however 
        DC_motors_setPower(0, 0);


        if (toleranceStart == 0)
        {
            toleranceStart = currentTime;
        }


        if (
            currentTime - toleranceStart
            >= SETTLE_TIME_MS
        )
        {
            controlActive = false;

            Serial.print("Turn complete. Heading: ");
            Serial.println(currentHeading);

            Serial2.print("Turn complete. Heading: ");
            Serial2.println(currentHeading);
        }


        previousError = currentError;
        previousTime = currentTime;

        return;
    }

    //reset the time wihtihn the tolerance zone to stop accumiulation when not there
    toleranceStart = 0;


    
    float derivative = (currentError - previousError) / dt;


    float output = KP * currentError + KD * derivative;


    // --------------------------------------------------------
    // CONVERT TO MOTOR POWER
    // --------------------------------------------------------

    int turnPower = abs((int)output);


    if (turnPower < MIN_TURN_POWER) {
        turnPower = MIN_TURN_POWER;
    }


    if (turnPower > MAX_TURN_POWER) {
        turnPower = MAX_TURN_POWER;
    }


    int direction;

    if (output > 0) {
        direction = 1;
    }
    else {
        direction = -1;
    }


    turnPower =
        turnPower
        * direction
        * TURN_SIGN;


    // Tracks run opposite directions for an on-the-spot turn
    DC_motors_setPower(
        turnPower,
        -turnPower
    );


    previousError = currentError;
    previousTime = currentTime;
}



void motor_control_stop()
{
    controlActive = false;

    DC_motors_setPower(0, 0);
}



bool motor_control_is_active()
{
    return controlActive;
}


float motor_control_get_target()
{
    return targetHeading;
}


float motor_control_get_error()
{
    return currentError;
}

void motor_control_set_kp(float kp)
{
    KP = kp;
}


void motor_control_set_kd(float kd)
{
    KD = kd;
}


float motor_control_get_kp()
{
    return KP;
}


float motor_control_get_kd()
{
    return KD;
}
