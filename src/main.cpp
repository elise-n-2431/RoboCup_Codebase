

#include <Arduino.h>
#include <Servo.h>
#include <DFRobot_MatrixLidar.h>

#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "outputs/motor_driver.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
#include "outputs/collection.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
#include "inputs/tof.h"
#include "inputs/xy_sensor.h"
#include "map.h"


// define global variables

void setup() {
    serial_init();
    // tof_init();
    // xy_init();
    pickup_servo_init();
    emag_init();
    motor_driver_init();
    collection_init();
    // proximity_init();
    // limit_switch_init();
}


void loop() {
    // tof_exe();
    //xy_exe();
    //display_map();

    // tof_visualize();

    // updateStateMachine();
    
    // // check inputs
    // proximity_exe();
    // limit_switch_exe();

    // // change outputs
    // servo_exe();
    // emag_exe();
    RobotCommand command = serial_get_command();


   switch (command)
    {
        

        case CMD_FORWARD:

            if (!collectionIsActive())
            {
                driveForward();
                Serial.println("Forward");
            }

            break;


       

        case CMD_REVERSE:

            if (!collectionIsActive())
            {
                driveReverse();
                Serial.println("Reverse");
            }

            break;


   


        case CMD_LEFT:

            if (!collectionIsActive())
            {
                turnLeft();
                Serial.println("Left");
            }

            break;



        case CMD_RIGHT:

            if (!collectionIsActive())
            {
                turnRight();
                Serial.println("Right");
            }

            break;




        case CMD_STOP:

            // Stop is always allowed, even during collection
            stopMotors();

            Serial.println("Stop");

            break;


       

        case CMD_SPEED_UP:

            increaseDrivePower();

            Serial.print("Drive power = ");
            Serial.println(getDrivePower());

            break;


       

        case CMD_SPEED_DOWN:

            decreaseDrivePower();

            Serial.print("Drive power = ");
            Serial.println(getDrivePower());

            break;


    

        case CMD_MOTORS_ON:

            armMotors();

            Serial.println("Motors ON");

            break;




        case CMD_MOTORS_OFF:

            // Always allowed for safety
            disarmMotors();

            Serial.println("Motors OFF");

            break;


  

        case CMD_COLLECT:

            if (!collectionIsActive())
            {
                Serial.println("Starting collection");

                collection_start();
            }
            else
            {
                Serial.println("Collection already running");
            }

            break;




        case CMD_NONE:
        default:
            break;
    }

    collection_exe();
    pickup_servo_exe();

}