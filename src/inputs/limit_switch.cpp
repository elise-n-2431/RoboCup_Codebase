#include "state_machine.h"
#include "limit_switch.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1



int limit_pin = 26;
const int CONSECUTIVE_HITS = 5;
int count_switch_on = 0;
int count_switch_off = 0;


void limit_switch_init()
{

    pinMode(
        limit_pin,
        INPUT_PULLUP
    );

    Serial.println(
        "Limit switch ready"
    );
}
void limit_switch_exe()
{
    // Only evaluate the limit switch when the crane
    // is stationary at one of the two pickup test positions.
    if (getCollectState() != VERT_REACHED &&
        getCollectState() != HORI_REACHED)
    {
        count_switch_on = 0;
        count_switch_off = 0;

        // Prevent an old contact from carrying into
        // the next pickup check.
        resetStateFlag(&STATE_FLAGS.magnet_hit);

        return;
    }


    if (digitalRead(limit_pin) == LOW)
    {
        count_switch_on++;
        count_switch_off = 0;
    }
    else
    {
        count_switch_on = 0;
        count_switch_off++;
    }


    if (count_switch_on >= CONSECUTIVE_HITS)
    {
        setStateFlag(&STATE_FLAGS.magnet_hit);

        count_switch_on = 0;
        count_switch_off = 0;
    }
    else if (count_switch_off >= CONSECUTIVE_HITS)
    {
        if (getCollectState() == VERT_REACHED)
        {
            setStateFlag(&STATE_FLAGS.no_vertical);
        }
        else if (getCollectState() == HORI_REACHED)
        {
            setStateFlag(&STATE_FLAGS.no_horizontal);
        }

        count_switch_on = 0;
        count_switch_off = 0;
    }
}

bool getLimitSwitch() {
    return digitalRead(limit_pin) == LOW;
}

void print_limit() {
    Serial.print("Limit ");
    Serial.println(getLimitSwitch());
}