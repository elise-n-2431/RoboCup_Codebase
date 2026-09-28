#include "state_machine.h"
#include "proximity.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const byte PROX_PIN = 20;

const int CONSECUTIVE_HITS = 8;
int count_metal = 0;
int count_dummy = 0;
int cutoff_var = 500;

void proximity_init() {
  pinMode(PROX_PIN, INPUT);
  
}

void proximity_exe()
{
    if (getNavState() != SORTING)
    {
        count_metal = 0;
        count_dummy = 0;
        return;
    }

    if (analogRead(PROX_PIN) < cutoff_var)
    {
        count_metal++;
        count_dummy = 0;
    }
    else
    {
        count_metal = 0;
        count_dummy++;
    }

    if (count_metal >= CONSECUTIVE_HITS)
    {
        Serial.println("Sorting: METAL detected");

        setStateFlag(&STATE_FLAGS.metal_identified);

        count_metal = 0;
        count_dummy = 0;
    }
    else if (count_dummy >= CONSECUTIVE_HITS)
    {
        Serial.println("Sorting: DUMMY detected");

        setStateFlag(&STATE_FLAGS.dummy_identified);

        count_metal = 0;
        count_dummy = 0;
    }
}

void print_proximity() {
    Serial.println(analogRead(PROX_PIN) < cutoff_var);
    Serial.println(analogRead(PROX_PIN));
}