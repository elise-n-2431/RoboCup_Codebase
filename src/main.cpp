

#include <Arduino.h>
#include <Servo.h>
#include "pickup_servo.h"
#include "lucas_driver.h"
#include "state_machine.h"


// define global variables


// define initial variables


void setup() {



}


void loop() {
    updateStateMachine();
    runBackgroundTasks();
}

void runBackgroundTasks() {

}