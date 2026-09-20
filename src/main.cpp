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
// #include "tasks.h"




static bool poseStreamEnabled = true;

static unsigned long lastPosePrintTime = 0;

const unsigned long POSE_PRINT_PERIOD_MS = 100;


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

    // imu_print_readings();

    pose_init();

    // imu_print_readings();
    navigator_start(true);

    xy_init();

    // tasks_init();   // last — starts the timebase from a clean point
}

// void loop()
// {
//     tasks_exe();
// }

int i = 0;
int max_iter = 20;

void loop()
{
    // PRINT STATEMENTS
    // pose_telemetry_exe();
    // print_state();
    // print_DC_power();
    // print_limit();

    xy_exe();
    // print_xy();

    imu_update();

    // imu_print_readings();
    tof_update();
    pose_update();
    // Serial.println("here!");
    // map_update();


    // if (i >= max_iter) {
    //     send_map_data();
    //     i = 0;
    // }
    // i ++;


    ultrasound_exe();
    limit_switch_exe();

    logic_exe();
    updateStateMachine();
    print_state();


    pickup_servo_exe();
    emag_exe();
    proximity_exe();

    pickup_servo_update();
    colour_sensor_update();
    smartservo_update();

    RobotCommand command = serial_exe();
    command_router_exe(command);

    navigator_exe();
    motor_control_update();

    // pose_print(Serial);

    

}

