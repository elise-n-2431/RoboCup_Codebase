

#include <Arduino.h>
#include <Servo.h>
#include <DFRobot_MatrixLidar.h>

#include "state_machine.h"
#include "logic_engine.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"

// define global variables


// define initial variables
void runBackgroundTasks() { 
    // these will raise a flag if a change in state is required
    // todo: frequency control

    // check inputs
    proximity_exe();
    limit_switch_exe();

    // change outputs
    servo_exe();
    emag_exe();    

}

void setup() {
    servo_init();
    emag_init();
    proximity_init();
    limit_switch_init();
}


void loop() {
    updateStateMachine();
    runBackgroundTasks();
}

