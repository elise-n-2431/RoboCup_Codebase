#include "driving_controller.h"

#include <Arduino.h>
#include <math.h>

#include "outputs/DC_motors.h"
#include "inputs/imu.h"



//to check if tunring or drving straight
enum MotorControlMode
{
    CONTROL_IDLE,
    CONTROL_TURNING,
    CONTROL_DRIVE_HEADING
};

//start in idle
static MotorControlMode controlMode = CONTROL_IDLE;

// To be tuned
static float TURN_KP = 7.0;

static float DRIVE_KP = 10.0;

const int MAX_DRIVE_CORRECTION = 100;

static int driveBasePower = 300;

// Minimum power for robot to actually rotate
const int MIN_TURN_POWER = 300;

const int MAX_TURN_POWER = 450;


// Consider target reached inside this angle
const float ANGLE_TOLERANCE = 2.5;


// Must stay in tolerance this long before finishing
const unsigned long SETTLE_TIME_MS = 100;


// If the robot turns away from the target,
// change this from +1 to -1
const int TURN_SIGN = 1;

const int DRIVE_STEER_SIGN = 1;





static float targetHeading = 0.0;
static float currentError = 0.0;
static float previousError = 0.0;

//varaibles used to chceck how long imu heading has been in tolerance zone
static unsigned long previousTime = 0;
static unsigned long toleranceStart = 0;

const unsigned long CONTROL_PERIOD_MS = 20;



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

void motor_control_set_target_heading(float heading)
{
    targetHeading =
        wrapHeading(heading);
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
    controlMode = CONTROL_IDLE;

    targetHeading = 0.0;
    currentError = 0.0;

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

    controlMode = CONTROL_TURNING;


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

    float currentHeading =
        imu_get_heading();

    currentError =
        headingError(
            targetHeading,
            currentHeading
        );

    previousTime = millis();
    toleranceStart = 0;

    controlMode = CONTROL_TURNING;
}


void motor_control_drive_current_heading(int basePower)
{
    targetHeading = imu_get_heading();

    driveBasePower = basePower;

    currentError = 0.0;

    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;


    Serial.print("Driving at heading: ");
    Serial.println(targetHeading);

    Serial.print("Base power: ");
    Serial.println(driveBasePower);

    Serial2.print("Driving at heading: ");
    Serial2.println(targetHeading);

    Serial2.print("Base power: ");
    Serial2.println(driveBasePower);
}

void motor_control_drive_heading(float heading, int basePower)
{
    targetHeading = wrapHeading(heading);
    driveBasePower = basePower;

    currentError =
        headingError(
            targetHeading,
            imu_get_heading()
        );

    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;
}



static void updateTurnControl(float currentHeading, unsigned long currentTime)
{
    currentError =
        headingError(
            targetHeading,
            currentHeading
        );

    //we have reached the imu heading wanted
    if (fabs(currentError) <= ANGLE_TOLERANCE)
    {
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
            controlMode = CONTROL_IDLE;

            Serial.print("Turn complete. Heading: ");
            Serial.println(currentHeading);
        }


        return;
    }


    toleranceStart = 0;


    

    float output =
        TURN_KP * currentError;


    int turnPower =
        abs((int)output);


    if (turnPower < MIN_TURN_POWER)
    {
        turnPower = MIN_TURN_POWER;
    }


    if (turnPower > MAX_TURN_POWER)
    {
        turnPower = MAX_TURN_POWER;
    }


    int direction;

    if (output > 0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }


    turnPower =
        turnPower
        * direction
        * TURN_SIGN;


    DC_motors_setPower(
        turnPower,
        -turnPower
    );
}


static void updateDriveHeadingControl(
    float currentHeading
)
{
    currentError =headingError(targetHeading, currentHeading);


    float correction = DRIVE_KP * currentError;
    correction *= DRIVE_STEER_SIGN;


    // Don't let heading correction become enormous
    if (correction > MAX_DRIVE_CORRECTION)
    {
        correction = MAX_DRIVE_CORRECTION;
    }

    if (correction < -MAX_DRIVE_CORRECTION)
    {
        correction = -MAX_DRIVE_CORRECTION;
    }


    int leftPower =
        driveBasePower + (int)correction;

    int rightPower =
        driveBasePower - (int)correction;


    DC_motors_setPower(
        leftPower,
        rightPower
    );
}

void motor_control_update()
{
    if (controlMode == CONTROL_IDLE)
    {
        return;
    }


    unsigned long currentTime =
        millis();


    if (
        currentTime - previousTime
        < CONTROL_PERIOD_MS
    )
    {
        return;
    }


    previousTime = currentTime;


    float currentHeading =
        imu_get_heading();


    if (controlMode == CONTROL_TURNING)
    {
        updateTurnControl(
            currentHeading,
            currentTime
        );

        return;
    }


    if (controlMode == CONTROL_DRIVE_HEADING)
    {
        updateDriveHeadingControl(
            currentHeading
        );

        return;
    }
}



void motor_control_stop()
{
    controlMode = CONTROL_IDLE;

    DC_motors_setPower(0, 0);
    Serial.println("Motor control stopped");
    Serial2.println("Motor control stopped");
}

bool motor_control_is_active()
{
    return controlMode != CONTROL_IDLE;
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
    TURN_KP = kp;
}


float motor_control_get_kp()
{
    return TURN_KP;
}




void motor_control_set_drive_kp(float kp)
{
    DRIVE_KP = kp;
}


float motor_control_get_drive_kp()
{
    return DRIVE_KP;
}

bool motor_control_is_turning()
{
    return controlMode == CONTROL_TURNING;
}


bool motor_control_is_driving()
{
    return controlMode == CONTROL_DRIVE_HEADING;
}