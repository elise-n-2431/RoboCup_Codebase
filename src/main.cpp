

#include <Arduino.h>
#include <Servo.h>
#include <DFRobot_MatrixLidar.h>
#include <Wire.h>
#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "outputs/motor_driver.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
//#include "outputs/collection.h"
#include "inputs/tof_expander.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
// #include "inputs/tof.h"
#include "inputs/xy_sensor.h"
#include "map.h"


// define global variables


// MAIN.CPP

void setup() {
    serial_init();
    // nav_tof_init();
    Wire.begin();
    Wire.setClock(100000);
    tof_init();
    // xy_init();
    // pickup_servo_init();
    // emag_init();
    motor_driver_init();
    // collection_init();
    // proximity_init();
    // limit_switch_init();
    Serial2.println("Testing123");
}




void loop() {

    //logic_exe();
    updateStateMachine();
    tof_update();

    // tof_exe();
    //xy_exe();
    //display_map();

    // tof_visualize();

    // // check inputs
    // proximity_exe();
    // limit_switch_exe();

    // // change outputs
    // servo_exe();
    // emag_exe();

    tof_print_readings();

    


    /*RobotCommand command = serial_get_command();


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
            Serial2.print("Drive power = ");
            Serial2.println(getDrivePower());
            break;


       

        case CMD_SPEED_DOWN:

            decreaseDrivePower();

            Serial.print("Drive power = ");
            Serial.println(getDrivePower());
            Serial2.print("Drive power = ");
            Serial2.println(getDrivePower());
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
                Serial2.println("Starting collection");
                collection_start();
            }
            else
            {
                Serial.println("Collection already running");
                Serial2.println("Collection already running");
            }

            break;

        case CMD_TOF_PRINT:
            nav_tof_print(Serial);
            nav_tof_print(Serial2);
            break;


        case CMD_TOF_STREAM_ON:

            frontTofStreamEnabled = true;

            // Makes the first set print immediately
            lastFrontTofPrintMs = millis() - FRONT_TOF_PRINT_PERIOD_MS;

            Serial.println("Front ToF streaming ON");
            Serial2.println("Front ToF streaming ON");
            break;


        case CMD_TOF_STREAM_OFF:

            frontTofStreamEnabled = false;

            Serial.println("Front ToF streaming OFF");
            Serial2.println("Front ToF streaming OFF");
            break;




        case CMD_NONE:
        default:
            break;
    }

    collection_exe();
    pickup_servo_exe();
    */

}


