#include "emag.h"
#include "state_machine.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

const byte EMAG_PIN = 26;

static bool emagState = false;

void emag_init()
{
    pinMode(EMAG_PIN, OUTPUT);
    emag_off();
}

void emag_on()
{
    digitalWrite(EMAG_PIN, HIGH);
    emagState = true;
}

void emag_off()
{
    digitalWrite(EMAG_PIN, LOW);
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