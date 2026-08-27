#include "emag.h"
#include "state_machine.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

const byte EMAG_PIN = 24;
const int DUTY = 200;
const int OFF = 0;

static bool emagState = false;

void emag_init() 
{
    pinMode(EMAG_PIN, OUTPUT);
    emag_off();
}

void emag_on()
{
    analogWrite(
        EMAG_PIN,
        DUTY
    );
    //digitalWrite(EMAG_PIN, HIGH);
    emagState = true;
}

void emag_off()
{
    analogWrite(
        EMAG_PIN,
        OFF
    );
    emagState = false;
}

bool emag_is_on()
{
    return emagState;
}

void emag_exe()
{
    switch (getCollectState()) {

        case LOWERING_VERT:
        case VERT_REACHED:
        case LOWERING_HORI:
        case HORI_REACHED:
        case PICKING_UP:
            emag_on();
            break;

        case RETURNING:
            emag_off();
            break;
        case DECIDING:
        case IDLE:
            emag_off();
            break;
    }
}