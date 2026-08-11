#include "emag.h"
#include "state_machine.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1

const byte EMAG_PIN = 26;

static bool emagState = false;

void emag_init()
{
    pinMode(EMAG_PIN, OUTPUT);
    emagOff();
}

void emagOn()
{
    digitalWrite(EMAG_PIN, HIGH);
    emagState = true;
}

void emagOff()
{
    digitalWrite(EMAG_PIN, LOW);
    emagState = false;
}

bool emagIsOn()
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
            emagOn();
            break;

        case RETURNING:
            emagOff();
            break;
        case DECIDING:
        case IDLE:
            emagOff();
            break;
    }
}