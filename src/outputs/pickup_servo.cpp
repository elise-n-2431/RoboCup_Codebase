//
// Created by elise on 4/07/2026.
//

#include "pickup_servo.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1
#include "state_machine.h"

Servo myservo;

const int vert_angle = 20;
const int hors_angle = 0;
const int dropoff_angle = 100;
const int idle_angle = 50;

void servo_init() {
    myservo.attach(1, 500, 2500);
}

void servo_exe() {
    if (STATE_FLAGS.state_changed) {
        switch (getCollectState()) {
            case IDLE:
                break;

            case COLLECT_VERTICAL:
                myservo.write(vert_angle);
                break;

            case COLLECT_HORISONTAL:
                myservo.write(hors_angle);
                break;
            
            case DROPPING:
                myservo.write(dropoff_angle);
                break;

            case RETURNING_TO_IDLE:
                myservo.write(idle_angle);
                break;
        }
    }
}

