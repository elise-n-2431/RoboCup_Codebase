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



void setup()
{
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
    motor_control_init();
    pickup_servo_init();
    emag_init();
    navigator_init();
    colour_sensor_init();
    pose_init();
    smartservo_init();

    delay(1000);
    ultrasound_init();
    smartservo_torque_on();
}


void loop()
{
    imu_update();
    tof_update();
    // pose_update();


    ultrasound_exe();
    //limit_switch_exe();

    logic_exe();
    updateStateMachine();


    pickup_servo_exe();
    // emag_exe();
    proximity_exe();

    pickup_servo_update();
    colour_sensor_update();
    smartservo_update();

    RobotCommand command = serial_exe();
    command_router_exe(command);

    navigator_exe();
    motor_control_update();

    //pose_telemetry_exe();
}


