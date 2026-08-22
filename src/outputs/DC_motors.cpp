#include "outputs/DC_motors.h"

#include <Arduino.h>
#include <Servo.h>

#include "comms/serial.h"


static Servo motorLeft;
static Servo motorRight;


// ============================================================
// MOTOR HARDWARE
// ============================================================

const int LEFT_PIN  = 30;
const int RIGHT_PIN = 31;


const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;


// Default power used for manual w/s/a/d control
static int drivePower = 250;


const int MIN_POWER  = 80;
const int MAX_POWER  = 450;
const int POWER_STEP = 25;


const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;


// From physical testing:
// commands below 1500 us were stronger,
// so reduce that side.
static float SCALING_FACTOR = 0.85;




static int manualLeftTrack = 0;
static int manualRightTrack = 0;




static int clampPulse(int pulse)
{
    if (pulse > FULL_FORWARD_US)
    {
        return FULL_FORWARD_US;
    }


    if (pulse < FULL_REVERSE_US)
    {
        return FULL_REVERSE_US;
    }


    return pulse;
}


static int clampPower(int power)
{
    if (power > MAX_POWER)
    {
        return MAX_POWER;
    }


    if (power < -MAX_POWER)
    {
        return -MAX_POWER;
    }


    return power;
}


static int clampDirection(int direction)
{
    if (direction > 0)
    {
        return 1;
    }


    if (direction < 0)
    {
        return -1;
    }


    return 0;
}


static void writeMotors(
    int leftPulse,
    int rightPulse
)
{
    motorLeft.writeMicroseconds(
        clampPulse(leftPulse)
    );

    motorRight.writeMicroseconds(
        clampPulse(rightPulse)
    );
}




static int makeMotorPulse(int motorPower)
{
    motorPower =
        clampPower(motorPower);


    if (motorPower == 0)
    {
        return STOP_US;
    }


    int direction;

    if (motorPower > 0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }


    int power =
        abs(motorPower);


    // Physical motor-driver correction
    if (direction < 0)
    {
        power =
            power * SCALING_FACTOR;
    }


    int pulse =
        STOP_US
        + direction * power;


    return clampPulse(pulse);
}



void DC_motors_setPower(
    int leftPower,
    int rightPower
)
{
    leftPower =
        clampPower(leftPower);

    rightPower =
        clampPower(rightPower);


    // Account for physical mounting direction
    int leftMotorPower =
        LEFT_INVERTED
        ? -leftPower
        : leftPower;


    int rightMotorPower =
        RIGHT_INVERTED
        ? -rightPower
        : rightPower;


    int leftPulse =
        makeMotorPulse(
            leftMotorPower
        );


    int rightPulse =
        makeMotorPulse(
            rightMotorPower
        );


    writeMotors(
        leftPulse,
        rightPulse
    );


    // TEMPORARY DEBUG
    /*
    Serial.print("Left pulse: ");
    Serial.print(leftPulse);

    Serial.print("   Right pulse: ");
    Serial.println(rightPulse);
    */
}




static void driveTracks(
    int leftTrack,
    int rightTrack
)
{
    leftTrack =
        clampDirection(leftTrack);

    rightTrack =
        clampDirection(rightTrack);


    DC_motors_setPower(
        leftTrack * drivePower,
        rightTrack * drivePower
    );
}




static void increaseDrivePower()
{
    drivePower += POWER_STEP;


    if (drivePower > MAX_POWER)
    {
        drivePower = MAX_POWER;
    }
}


static void decreaseDrivePower()
{
    drivePower -= POWER_STEP;


    if (drivePower < MIN_POWER)
    {
        drivePower = MIN_POWER;
    }
}


int getDrivePower()
{
    return drivePower;
}




static void applyCommand(
    RobotCommand command
)
{
    switch (command)
    {
        case CMD_FORWARD:

            manualLeftTrack = +1;
            manualRightTrack = +1;

            break;


        case CMD_REVERSE:

            manualLeftTrack = -1;
            manualRightTrack = -1;

            break;


        case CMD_LEFT:

            manualLeftTrack = -1;
            manualRightTrack = +1;

            break;


        case CMD_RIGHT:

            manualLeftTrack = +1;
            manualRightTrack = -1;

            break;


        case CMD_STOP:

            manualLeftTrack = 0;
            manualRightTrack = 0;

            break;


        case CMD_SPEED_UP:

            increaseDrivePower();

            break;


        case CMD_SPEED_DOWN:

            decreaseDrivePower();

            break;


        default:

            break;
    }
}



void DC_motors_init()
{
    motorLeft.attach(
        LEFT_PIN
    );

    motorRight.attach(
        RIGHT_PIN
    );


    DC_motors_setPower(
        0,
        0
    );
}




void DC_motors_exe(RobotCommand command)
{
    applyCommand(command);

    driveTracks(
        manualLeftTrack,
        manualRightTrack
    );
}