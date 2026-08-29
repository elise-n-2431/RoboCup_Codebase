#include "outputs/DC_motors.h"

#include <Arduino.h>
#include <Servo.h>



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


const int MAX_POWER  = 450;

const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;

// From physical testing:
// commands below 1500 us were stronger,
// so reduce that side.
static float SCALING_FACTOR = 0.85;




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

