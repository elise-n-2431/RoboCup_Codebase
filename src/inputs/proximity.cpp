#include "state_machine.h"
#include "sensors.h"
#include "proximity.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

const byte PROX_PIN = 20;
const byte PROX_PIN_2 = 14;

void proximity_init() {
  pinMode(PROX_PIN_2, INPUT);
  pinMode(PROX_PIN, INPUT);
  
}

void proximity_exe() {
    if (analogRead(PROX_PIN) > 500) {
        setSensorFlag(&SENSOR_FLAGS.metal_detected);
    } else {
        setSensorFlag(&SENSOR_FLAGS.metal_detected);
    }
}