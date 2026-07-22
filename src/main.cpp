

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

void setup() {
    servo_init();
    emag_init();
    proximity_init();
    limit_switch_init();
}


void loop() {
    updateStateMachine();
    
    // check inputs
    proximity_exe();
    limit_switch_exe();

    // change outputs
    servo_exe();
    emag_exe();    

}

