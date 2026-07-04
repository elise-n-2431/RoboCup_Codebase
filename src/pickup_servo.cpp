//
// Created by elise on 4/07/2026.
//

#include "pickup_servo.h"
#include <Arduino.h>
#include <Servo.h>

Servo myservoA, myservoB, myservoC, myservoD;

// Global variable defining the turn amount (in degrees)
int TURN_ANGLE = 15;

// Center/neutral position for the servos
const int CENTER_POS = 90;

void servo_init() {
    myservoA.attach(2);  // attaches the servo to the servo object using pin 2
    myservoB.attach(3);  // attaches the servo to the servo object using pin 3
    myservoC.attach(4);  // attaches the servo to the servo object using pin 4
    myservoD.attach(5);  // attaches the servo to the servo object using pin 5
}

void servo_exe() {
    // Turn +TURN_ANGLE degrees from center
    myservoA.write(CENTER_POS + TURN_ANGLE);
    myservoB.write(CENTER_POS + TURN_ANGLE);
    myservoC.write(CENTER_POS + TURN_ANGLE);
    myservoD.write(CENTER_POS + TURN_ANGLE);
    delay(1500);                           // waits for the servo to get there

    // Turn back -TURN_ANGLE degrees from center (i.e. the opposite direction)
    myservoA.write(CENTER_POS - TURN_ANGLE);
    myservoB.write(CENTER_POS - TURN_ANGLE);
    myservoC.write(CENTER_POS - TURN_ANGLE);
    myservoD.write(CENTER_POS - TURN_ANGLE);
    delay(1500);
}