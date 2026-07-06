//
// Created by elise on 4/07/2026.
//

#include "pickup_servo.h"
#include <Arduino.h>
#include <Servo.h>

Servo myservoA;

// Global variable defining the turn amount (in degrees)
int pos = 0;

// Center/neutral position for the servos
// const int CENTER_POS = 90;

void servo_init() {
    myservoA.attach(1, 500, 2500);  // attaches the servo to the servo object using pin 2
    Serial.println("Initializing Servo \n");
}

void servo_exe() {
    myservoA.write(0);
    delay(2000);                          

    pos = myservoA.read();
    Serial.print("Servo position 1: ");
    Serial.println(pos);

    myservoA.write(45);
    delay(2000);

    pos = myservoA.read(); 
    Serial.print("Servo position 2: ");
    Serial.println(pos);

    myservoA.write(50);
    delay(2000);

    pos = myservoA.read(); 
    Serial.print("Servo position 3: ");
    Serial.println(pos);

}