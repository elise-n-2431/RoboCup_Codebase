#include "state_machine.h"
#include "proximity.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const byte PROX_PIN = 20;

const int CONSECUTIVE_HITS = 8;

const int METAL_CONSECUTIVE_HITS = 5;
const int DUMMY_CONSECUTIVE_HITS = 14;
int count_metal = 0;
int count_dummy = 0;
int cutoff_var = 500;

void proximity_init() {
  pinMode(PROX_PIN, INPUT);
  
}

void proximity_exe()
{
    // Only classify a weight while the robot is in SORTING.
    // Reset evidence whenever we leave this state so old
    // samples cannot carry into the next weight.
    if (getNavState() != SORTING)
    {
        count_metal = 0;
        count_dummy = 0;
        return;
    }

    // Read once and use the same value for this whole update.
    int proxValue =
        analogRead(PROX_PIN);

    // Useful for checking how close the real weights are
    // to the threshold during competition testing.
    Serial.print("SORT_PROX,");
    Serial.println(proxValue);

    // ----------------------------------------------------
    // METAL
    //
    // Lower ADC value currently means metal detected.
    //
    // We allow metal to confirm relatively quickly because
    // keeping a real weight is preferable to accidentally
    // rejecting it as a dummy.
    // ----------------------------------------------------
    if (proxValue < cutoff_var)
    {
        count_metal++;
        count_dummy = 0;

        if (count_metal >= METAL_CONSECUTIVE_HITS)
        {
            Serial.println(
                "Sorting: METAL detected"
            );

            setStateFlag(
                &STATE_FLAGS.metal_identified
            );

            count_metal = 0;
            count_dummy = 0;

            return;
        }
    }

    // ----------------------------------------------------
    // DUMMY
    //
    // Be more conservative before throwing a weight away.
    // A real weight that is slightly poorly positioned over
    // the proximity sensor should not immediately be rejected.
    // ----------------------------------------------------
    else
    {
        count_dummy++;
        count_metal = 0;

        if (count_dummy >= DUMMY_CONSECUTIVE_HITS)
        {
            Serial.println(
                "Sorting: DUMMY detected"
            );

            setStateFlag(
                &STATE_FLAGS.dummy_identified
            );

            count_metal = 0;
            count_dummy = 0;

            return;
        }
    }
}


void print_proximity() {
    Serial.println(analogRead(PROX_PIN) < cutoff_var);
    Serial.println(analogRead(PROX_PIN));
}