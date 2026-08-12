#include "DC_motors.h"

#include <Arduino.h>
#include <Servo.h>
#include "state_machine.h"
#include "comms/serial.h"
// #include "navigator.h"

static Servo motorLeft;
static Servo motorRight;

const int LEFT_PIN  = 30;
const int RIGHT_PIN = 31;

const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;

static int drivePower = 250;

const int MIN_POWER  = 80;
const int MAX_POWER  = 430;
const int POWER_STEP = 25;

const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;

static int manualLeftTrack = 0;
static int manualRightTrack = 0;

static void increaseDrivePower()
{
    drivePower += POWER_STEP;
    if (drivePower > MAX_POWER) drivePower = MAX_POWER;
}

static void decreaseDrivePower()
{
    drivePower -= POWER_STEP;
    if (drivePower < MIN_POWER) drivePower = MIN_POWER;
}

static void applyCommand(RobotCommand command)
{
    switch (command)
    {
        case CMD_FORWARD: manualLeftTrack = +1; manualRightTrack = +1; break;
        case CMD_REVERSE: manualLeftTrack = -1; manualRightTrack = -1; break;
        case CMD_LEFT:    manualLeftTrack = -1; manualRightTrack = +1; break;
        case CMD_RIGHT:   manualLeftTrack = +1; manualRightTrack = -1; break;
        case CMD_STOP:    manualLeftTrack =  0; manualRightTrack =  0; break;
        case CMD_SPEED_UP:   increaseDrivePower(); break;
        case CMD_SPEED_DOWN: decreaseDrivePower(); break;
        default: break;
    }
}

static int clampPulse(int pulse)
{
    if (pulse > FULL_FORWARD_US) return FULL_FORWARD_US;
    if (pulse < FULL_REVERSE_US) return FULL_REVERSE_US;
    return pulse;
}

static int clampDirection(int direction)
{
    if (direction > 0) return 1;
    if (direction < 0) return -1;
    return 0;
}

static void writeMotors(int leftPulse, int rightPulse)
{
    motorLeft.writeMicroseconds(clampPulse(leftPulse));
    motorRight.writeMicroseconds(clampPulse(rightPulse));
}

static void driveTracks(int leftTrack, int rightTrack)
{
    leftTrack  = clampDirection(leftTrack);
    rightTrack = clampDirection(rightTrack);

    int leftSign  = LEFT_INVERTED  ? -leftTrack  : leftTrack;
    int rightSign = RIGHT_INVERTED ? -rightTrack : rightTrack;

    writeMotors(STOP_US + leftSign * drivePower, STOP_US + rightSign * drivePower);
}

static void stopMotors()
{
    writeMotors(STOP_US, STOP_US);
}

void DC_motors_init()
{
    motorLeft.attach(LEFT_PIN);
    motorRight.attach(RIGHT_PIN);
    stopMotors();
}

int getDrivePower()
{
    return drivePower;
}

void DC_motors_exe(DriveMode mode)
{
    if (getNavState() == REVERSING) {
        driveTracks(-1, -1);
        return;
    }

    if (mode == DRIVE_SERIAL) {
        applyCommand(serial_get_command());
        driveTracks(manualLeftTrack, manualRightTrack);
        return;
    }

    // NavCommand command = navigator_getCommand();
    // driveTracks(command.leftTrack, command.rightTrack);
}