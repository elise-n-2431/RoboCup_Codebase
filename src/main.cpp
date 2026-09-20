#include <Arduino.h>
#include <Wire.h>
#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "comms/flag_control.h"
#include "outputs/DC_motors.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
#include "outputs/smart_servo.h"
#include "inputs/tof_expander.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
#include "inputs/xy_sensor.h"
#include "navigator.h"
#include "driving_controller.h"
#include "inputs/encoders.h"
#include "pose.h"
#include "inputs/colour_sensor.h"
#include "inputs/imu.h"
#include "inputs/ultrasound.h"
#include "comms/command_router.h"
#include "map.h"


const byte GO_PIN = 26;

bool run = false;


static bool poseStreamEnabled = true;
static unsigned long lastPosePrintTime = 0;

const unsigned long POSE_PRINT_PERIOD_MS = 100;


void setup()
{
    // GO button
    pinMode(GO_PIN, INPUT);

    // Communications
    serial_init();
    encoders_init();

    // Hardware
    DC_motors_init();

    imu_init();

    tof_init();
    limit_switch_init();
    proximity_init();

    // Control
    map_init();
    motor_control_init();
    pickup_servo_init();
    emag_init();
    navigator_init();
    colour_sensor_init();
    smartservo_init();

    delay(1000);

    smartservo_torque_on();
    ultrasound_init();

    pose_init();

    // Start in roaming-only mode.
    // Type "auto" to enable weight pickup.
    navigator_start(true);

    xy_init();
}


int i = 0;
int max_iter = 200;


void loop()
{

    if (digitalRead(GO_PIN) == HIGH)
    {
        run = true;
    }

    xy_exe();

    imu_update();
    tof_update();

    ultrasound_exe();
    limit_switch_exe();

    colour_sensor_update();

    //print_limit();
    // print_xy();
    // tof_print_readings(Serial);


    RobotCommand command = serial_exe();
    command_router_exe(command);


    if (!run)
    {
        return;
    }


    logic_exe();
    updateStateMachine();


   
    pickup_servo_exe();
    emag_exe();
    proximity_exe();

    pickup_servo_update();
    smartservo_update();
    navigator_exe();
    motor_control_update();


    // Optional
    pose_update();
    // map_update();

    /*
    if (i >= max_iter)
    {
        send_map_data();
        i = 0;
    }

    i++;
    */
}