#include "state_machine.h"
#include "proximity.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const byte PROX_PIN = 20;
const byte PROX_PIN_2 = 14;

const int CONSECUTIVE_HITS = 50;
int count_metal = 0;
int count_dummy = 0;


void proximity_init() {
  pinMode(PROX_PIN_2, INPUT);
  pinMode(PROX_PIN, INPUT);
  
}

// should involve centring
void proximity_exe() {
    if (analogRead(PROX_PIN) > 500) {
        count_metal += 1;
        count_dummy = 0;
    } else {
        count_metal = 0;
        count_dummy += 1;
    }

    if (count_metal >= 50) {
        setStateFlag(&STATE_FLAGS.metal_identified);
        count_metal = 0;
        count_dummy = 0;

    } else if (count_dummy >= 50) {
        setStateFlag(&STATE_FLAGS.dummy_identified);
        count_metal = 0;
        count_dummy = 0;
    }
}

