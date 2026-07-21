#include "state_machine.h"
#include "../inputs/sensors.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1

const byte EMAG_PIN = 26;


void emag_init() {
    pinMode(EMAG_PIN, OUTPUT);

}


void emag_exe() {
    switch (getCurrentState()) {

        case IDLE:
            digitalWrite(EMAG_PIN, LOW);
            break;

        case COLLECT_VERTICAL:
            digitalWrite(EMAG_PIN, HIGH);
            break;

        case COLLECT_HORISONTAL:
            digitalWrite(EMAG_PIN, HIGH);
            break;

        
        case DROPPING:
            digitalWrite(EMAG_PIN, HIGH);
            break;


        case RETURNING_TO_IDLE:
            digitalWrite(EMAG_PIN, LOW);
            break;

    }

}