#include <Arduino.h>
#include <Wire.h>
#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "comms/flag_control.h"
#include "outputs/DC_motors.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
// #include "inputs/nav_tof.h"
//#include "outputs/collection.h"
#include "inputs/tof_expander.h"
#include "inputs/imu.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
#include "inputs/xy_sensor.h"
#include "navigator.h"
#include "driving_controller.h"
#include "inputs/encoders.h"
#include "pose.h"

static bool tofStreamEnabled = false;

static unsigned long lastTofPrintTime = 0;

const unsigned long TOF_PRINT_PERIOD_MS = 2500;

// Main.cpp 
// Contains the executable and initialisation files for the purpose of implementing a task scheduler

/*void setup() {
    serial_init();
    Wire.begin();
    Wire.setClock(100000);

    //inputs
    imu_init();
    tof_init();
    //limit_switch_init();
    //proximity_init();
    //xy_init();

    //outputs
    //DC_motors_init();
    pickup_servo_init();
    emag_init();

    // logic
    // navigator_init(); For later
    //proximity_init();
}

void loop() {
    //logic_exe();
    //updateStateMachine();
    // navigator_exe(); For later

    //outputs
    //DC_motors_exe(DRIVE_SERIAL);
    //pickup_servo_exe(); // does it also need to call update?
    //emag_exe();

    //inputs
    imu_update();
    tof_update();
    tof_print_readings();
    delay(3000);
    //limit_switch_exe();
    //proximity_exe();
    //xy_exe();

}*/

void handleRobotCommand(RobotCommand command)
{
    switch (command)
    {
        // ====================================================
        // MANUAL MOTOR COMMANDS
        // ====================================================

        case CMD_FORWARD:
        case CMD_REVERSE:
        case CMD_LEFT:
        case CMD_RIGHT:
        case CMD_STOP:
        case CMD_SPEED_UP:
        case CMD_SPEED_DOWN:

            if (
                !navigator_is_active() &&
                !motor_control_is_active()
            )
            {
                DC_motors_exe(command);
            }

            break;


        // ====================================================
        // TOF
        // ====================================================

        case CMD_TOF_PRINT:
            Serial2.println("ToF reached");
            tof_print_readings(Serial2);
            tof_print_readings(Serial);

            break;


        case CMD_TOF_STREAM_ON:

            tofStreamEnabled = true;

            lastTofPrintTime =
                millis() - TOF_PRINT_PERIOD_MS;

            Serial.println("ToF streaming ON");
            Serial2.println("ToF streaming ON");

            break;


        case CMD_TOF_STREAM_OFF:

            tofStreamEnabled = false;

            Serial.println("ToF streaming OFF");
            Serial2.println("ToF streaming OFF");

            break;


        // ====================================================
        // LATER
        // ====================================================

        case CMD_COLLECT:

            Serial.println(
                "Collection command received"
            );

            break;


        case CMD_MOTORS_ON:

            Serial.println(
                "Motor enable command received"
            );

            break;


        case CMD_MOTORS_OFF:

            motor_control_stop();
            navigator_stop();

            DC_motors_setPower(0, 0);

            Serial.println(
                "Motors stopped"
            );

            break;


        case CMD_NONE:
        default:

            break;
    }
}

void printRobotTelemetry(Stream &port)
{
    port.print("ROBOT,");

    port.print(pose_get_x_mm());
    port.print(",");

    port.print(pose_get_y_mm());
    port.print(",");

    port.print(pose_get_heading_deg());
    port.print(",");

    // Navigation ToFs
    port.print(tof_get_nav_outer_left());
    port.print(",");

    port.print(tof_get_nav_inner_left());
    port.print(",");

    port.print(tof_get_nav_inner_right());
    port.print(",");

    port.print(tof_get_nav_outer_right());
    port.print(",");

    // Weight ToFs
    port.print(tof_get_weight_left_top());
    port.print(",");

    port.print(tof_get_weight_left_bottom());
    port.print(",");

    port.print(tof_get_weight_right_top());
    port.print(",");

    port.println(tof_get_weight_right_bottom());
}

void setup()
{
    // Communications
    serial_init();
    encoders_init();

    // Hardware
    DC_motors_init();

    imu_init();

    tof_init();

    // Control
    motor_control_init();

    navigator_init();

    pose_init();

    Serial.println(
        "Navigation and heading control ready"
    );

    Serial2.println(
        "Navigation and heading control ready"
    );

    Serial2.println(
        "Type help for commands"
    );
}

static unsigned long lastPosePrint = 0;


static bool poseStreamEnabled = true;

static unsigned long lastPosePrintTime = 0;

const unsigned long POSE_PRINT_PERIOD_MS = 100;

void loop()
{
    // Latest heading
    imu_update();
    tof_update();
    pose_update();
    if (poseStreamEnabled && millis() - lastPosePrintTime >= POSE_PRINT_PERIOD_MS)
    {
    lastPosePrintTime = millis();
    printRobotTelemetry(Serial2);
    }
    // USB + Bluetooth commands
    RobotCommand command =
        serial_exe();


    handleRobotCommand(
        command
    );

    // Autonomous navigator
    if (navigator_is_active())
    {
        navigator_exe();
    }

    // Run heading feedback control
    motor_control_update();

    if (
        tofStreamEnabled &&
        millis() - lastTofPrintTime
        >= TOF_PRINT_PERIOD_MS
    )
    {
        lastTofPrintTime = millis();

        tof_print_readings(Serial);
        tof_print_readings(Serial2);
    }
}


