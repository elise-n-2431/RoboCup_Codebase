#include "DC_motors.h"

#include <Arduino.h>
#include <Servo.h>
#include "state_machine.h"
#include "comms/serial.h"
#include "navigator.h"

static Servo motorLeft;
static Servo motorRight;

const int LEFT_PIN  = 30;
const int RIGHT_PIN = 31;

const int STOP_US         = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;

static int drivePower = 250;

const int MIN_POWER  = 280;
const int MAX_POWER  = 430;
const int POWER_STEP = 25;

const bool LEFT_INVERTED  = false;
const bool RIGHT_INVERTED = true;

static int manualLeftTrack = 0;
static int manualRightTrack = 0;

/*to try and make the motors even as they turn faster when below 1500
 then above so if the number sent will be under 1500 it will be scaled less */


static float SCALING_FACTOR = 0.85;

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

//allows for a number between 0 and 1 to be sent to the motor for steering 
static int makeMotorPulse(int motorPower)
{
    if (motorPower > MAX_POWER) {
        motorPower = MAX_POWER;
    }

    if (motorPower < -MAX_POWER) {
        motorPower = -MAX_POWER;
    }

    if (motorPower == 0) {
        return STOP_US;
    }

    int direction;

    if (motorPower > 0) {
        direction = 1;
    }
    else {
        direction = -1;
    }

    int power = abs(motorPower);

    // Scales the side below 1500 as found with the testing
    if (direction < 0) {
        power = power * SCALING_FACTOR;
    }

    return clampPulse(
        STOP_US + direction * power
    );
}



void DC_motors_setPower(int leftPower, int rightPower)
{
    int leftMotorPower =
        LEFT_INVERTED ? -leftPower : leftPower;

    int rightMotorPower =
        RIGHT_INVERTED ? -rightPower : rightPower;

    int leftPulse = makeMotorPulse(leftMotorPower);
    int rightPulse = makeMotorPulse(rightMotorPower);
    writeMotors(
        leftPulse,
        rightPulse
    );

    Serial.print("Left pulse: ");
    Serial.print(leftPulse);

    Serial.print("   Right pulse: ");
    Serial.println(rightPulse);

    Serial2.print("Left pulse: ");
    Serial2.print(leftPulse);

    Serial2.print("   Right pulse: ");
    Serial2.println(rightPulse);
}


static void driveTracks(int leftTrack, int rightTrack)
{
    leftTrack  = clampDirection(leftTrack);
    rightTrack = clampDirection(rightTrack);

    int leftSign  = LEFT_INVERTED  ? -leftTrack  : leftTrack;
    int rightSign = RIGHT_INVERTED ? -rightTrack : rightTrack;

    DC_motors_setPower(
        leftTrack * drivePower,
        rightTrack * drivePower
    );
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

    NavCommand command = navigator_getCommand();
    //driveTracks(command.leftTrack, command.rightTrack);
}