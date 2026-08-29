#include "state_machine.h"
#include "limit_switch.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1

const byte SX1509_LIMIT_ADDRESS = 0x3E;
const byte AIO5_PIN = 5;
SX1509 io;

const int CONSECUTIVE_HITS = 50;
int count_switch_on = 0;
int count_switch_off = 0;


void limit_switch_init()
{
    if (!io.begin(SX1509_LIMIT_ADDRESS))
    {
        Serial.println(
            "ERROR: limit switch SX1509 not found"
        );

        return;
    }

    io.pinMode(
        AIO5_PIN,
        INPUT_PULLUP
    );

    Serial.println(
        "Limit switch ready"
    );
}

void limit_switch_exe() {
    static unsigned long lastSwitchPrint = 0;

    if (millis() - lastSwitchPrint >= 250)
    {
        lastSwitchPrint = millis();

        Serial.print("LIMIT RAW = ");
        Serial.println(io.digitalRead(AIO5_PIN));
    }
    if (getCollectState() != VERT_REACHED &&
        getCollectState() != HORI_REACHED) {
        count_switch_on = 0;
        count_switch_off = 0;
        return;
    }

    if (io.digitalRead(AIO5_PIN) == LOW) {
        count_switch_on++;
        count_switch_off = 0;
    } else {
        count_switch_on = 0;
        count_switch_off++;
    }

    if (count_switch_on >= 50) {
        setStateFlag(&STATE_FLAGS.magnet_hit);
        count_switch_on = 0;
        count_switch_off = 0;
    }
    else if (count_switch_off >= 50) {
        if (getCollectState() == VERT_REACHED) {
            setStateFlag(&STATE_FLAGS.no_vertical);
        }
        else if (getCollectState() == HORI_REACHED) {
            setStateFlag(&STATE_FLAGS.no_horizontal);
        }

        count_switch_on = 0;
        count_switch_off = 0;
    }
}

bool getLimitSwitch() {
    return io.digitalRead(AIO5_PIN) == LOW;
}