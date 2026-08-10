#include "motor_driver.h"

#include <Arduino.h>
#include <Servo.h>


// ============================================================
// MOTOR OBJECTS
// ============================================================

static Servo motorLeft;
static Servo motorRight;


// ============================================================
// HARDWARE CONFIGURATION
// ============================================================

// Dual DC motor controller connected through SERIAL1 connector
const int LEFT_PIN  = 0;
const int RIGHT_PIN = 1;


// Servo pulse values expected by motor controller
const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;


// ============================================================
// DRIVE SETTINGS
// ============================================================

static int drivePower = 250;

const int MIN_POWER  = 80;
const int MAX_POWER  = 430;
const int POWER_STEP = 25;


// Motors face opposite physical directions
const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;


// ============================================================
// INTERNAL STATE
// ============================================================

static bool motorsArmed = false;


// ============================================================
// PRIVATE FUNCTIONS
// ============================================================

static int clampPulse(int pulse)
{
    if (pulse > FULL_FORWARD_US) {
        return FULL_FORWARD_US;
    }

    if (pulse < FULL_REVERSE_US) {
        return FULL_REVERSE_US;
    }

    return pulse;
}


static void writeMotors(int leftPulse, int rightPulse)
{
    motorLeft.writeMicroseconds(clampPulse(leftPulse));
    motorRight.writeMicroseconds(clampPulse(rightPulse));
}


// Makes sure direction is only -1, 0 or +1
static int clampDirection(int direction)
{
    if (direction > 0) {
        return 1;
    }

    if (direction < 0) {
        return -1;
    }

    return 0;
}


// ============================================================
// INITIALISATION
// ============================================================

void motor_driver_init()
{
    motorLeft.attach(LEFT_PIN);
    motorRight.attach(RIGHT_PIN);

    stopMotors();
}


// ============================================================
// TRACK CONTROL
// ============================================================

void driveTracks(int leftTrack, int rightTrack)
{
    if (!motorsArmed) {
        stopMotors();
        return;
    }

    leftTrack  = clampDirection(leftTrack);
    rightTrack = clampDirection(rightTrack);


    // Account for opposite physical orientation of motors
    int leftSign =
        LEFT_INVERTED ? -leftTrack : leftTrack;

    int rightSign =
        RIGHT_INVERTED ? -rightTrack : rightTrack;


    int leftPulse =
        STOP_US + leftSign * drivePower;

    int rightPulse =
        STOP_US + rightSign * drivePower;


    writeMotors(leftPulse, rightPulse);
}


// ============================================================
// HIGH-LEVEL MOVEMENT COMMANDS
// ============================================================

void driveForward()
{
    driveTracks(+1, +1);
}


void driveReverse()
{
    driveTracks(-1, -1);
}


void turnLeft()
{
    driveTracks(-1, +1);
}


void turnRight()
{
    driveTracks(+1, -1);
}


void stopMotors()
{
    writeMotors(STOP_US, STOP_US);
}


// ============================================================
// MOTOR ARM / DISARM
// ============================================================

void armMotors()
{
    motorsArmed = true;

    // Always begin stationary
    stopMotors();
}


void disarmMotors()
{
    stopMotors();

    motorsArmed = false;
}


bool motorsAreArmed()
{
    return motorsArmed;
}


// ============================================================
// SPEED CONTROL
// ============================================================

void increaseDrivePower()
{
    drivePower += POWER_STEP;

    if (drivePower > MAX_POWER) {
        drivePower = MAX_POWER;
    }
}


void decreaseDrivePower()
{
    drivePower -= POWER_STEP;

    if (drivePower < MIN_POWER) {
        drivePower = MIN_POWER;
    }
}


int getDrivePower()
{
    return drivePower;
}