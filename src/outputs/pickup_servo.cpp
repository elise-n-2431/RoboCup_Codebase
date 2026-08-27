//
// Created by elise on 4/07/2026.
//

#include "pickup_servo.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1
#include "state_machine.h"

Servo craneServo;

const int CRANE_SERVO_PIN = 29;

// Start conservative and calibrate these
const int IDLE_US       = 1500;
const int VERTICAL_US   = 1050;
const int HORIZONTAL_US = 950;
const int DROPOFF_US    = 1950;

static int currentPulse = IDLE_US;
static int targetPulse  = IDLE_US;

static int servoStepUs = 5;

const int SLOW_STEP_US   = 30;
const int MEDIUM_STEP_US = 37;
const int FAST_STEP_US   = 50;

const unsigned long SERVO_UPDATE_MS = 10;

static unsigned long lastServoUpdate = 0;

static void moveCraneTo(int target, int speed);

void pickup_servo_init()
{
    craneServo.attach(CRANE_SERVO_PIN, 500, 2500);
    craneIdle();
}

void craneIdle()
{
    moveCraneTo(IDLE_US, FAST_STEP_US);
}

void craneVertical()
{
    // Slowly lower towards the weight
    moveCraneTo(VERTICAL_US, SLOW_STEP_US);
}

void craneHorizontal()
{
    // Also relatively slow near the weight
    moveCraneTo(HORIZONTAL_US, SLOW_STEP_US);
}

void craneDrop()
{
    // Lift weight into robot quickly
    moveCraneTo(DROPOFF_US, FAST_STEP_US);
}

static void moveCraneTo(int target, int speed)
{
    targetPulse = target;
    servoStepUs = speed;
}

void pickup_servo_exe()
{
    switch(getCollectState()) {

        case LOWERING_VERT:
            craneVertical();
            break;

        case VERT_REACHED:
            // stay stationary
            break;

        case LOWERING_HORI:
            craneHorizontal();
            break;

        case HORI_REACHED:
            // stay stationary
            break;

        case PICKING_UP:
            craneDrop();
            break;

        case RETURNING:
            craneIdle();
            break;

        case DECIDING:
            // stay stationary
            break;

        case IDLE:
            // stay stationary
            break;
    }
}

// Ramps currentPulse toward targetPulse at servoStepUs per SERVO_UPDATE_MS.
// Call every loop() tick, independent of pickup_servo_exe(), so movement
// stays smooth regardless of how often collect state changes.
void pickup_servo_update()
{
    unsigned long now = millis();

    if (now - lastServoUpdate < SERVO_UPDATE_MS) {
        return;
    }

    lastServoUpdate = now;

    if (currentPulse < targetPulse)
    {
        currentPulse += servoStepUs;

        if (currentPulse > targetPulse) {
            currentPulse = targetPulse;
        }
    }
    else if (currentPulse > targetPulse)
    {
        currentPulse -= servoStepUs;

        if (currentPulse < targetPulse) {
            currentPulse = targetPulse;
        }
    }

    craneServo.writeMicroseconds(currentPulse);
}