#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "comms/flag_control.h"
#include "comms/command_router.h"

#include "outputs/DC_motors.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
#include "outputs/smart_servo.h"

#include "inputs/tof_expander.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
#include "inputs/xy_sensor.h"
#include "inputs/encoders.h"
#include "inputs/colour_sensor.h"
#include "inputs/imu.h"
#include "inputs/ultrasound.h"

#include "navigator.h"
#include "driving_controller.h"
#include "pose.h"
#include "map.h"
#include "arena_config.h"
#include "debug_print.h"
#include "priority_targets.h"
#include "competition_setup.h"

const byte GO_PIN = 26;

bool run = false;
static bool previousGoHigh = false;
//static int mapPrintCounter = 0;

void setup()
{
    pinMode(GO_PIN, INPUT);

    serial_init();
    encoders_init();
    DC_motors_init();
    imu_init();
    tof_init();
    limit_switch_init();


    proximity_init();

    motor_control_init();
    pickup_servo_init();
    emag_init();
    navigator_init();
    colour_sensor_init();
    smartservo_init();

    delay(1000);

    smartservo_torque_on();
    ultrasound_init();
    xy_init();

    pose_init();
    map_init();

    priority_targets_load_starting_weights();
    // Serial.println("Loaded starting weight coordinates:");
    priority_targets_print(Serial);
    //map_init();


    // Serial.println("RUN,WAITING");
    Serial2.println("RUN,WAITING");
}

static void checkGo()
{
    const bool high = digitalRead(GO_PIN) == HIGH;
    const bool pressed = high && !previousGoHigh;
    previousGoHigh = high;

    if (!pressed || arena_run_started()) return;

    if (!imu_is_online() ||
        !isfinite(imu_get_heading()) ||
        !colour_sensor_capture_home()) {
        // Serial.println("ERR,GO,sensors_not_ready");
        Serial2.println("ERR,GO,sensors_not_ready");
        return;
    }

    // Robot must be in its configured starting pose, on its selected base.
    // Re-sample the actual base colour; do not invent RGB thresholds.
    imu_update();

    // Competition assumption:
    // robot physically starts inside its configured home base.
    
    pose_reset();
    // arena_set_start_pose(300, 300, 0);

    if (!navigator_start(arena_get_config().pickupEnabled))
    {
        // Serial.println("ERR,GO,navigator_start");
        Serial2.println("ERR,GO,navigator_start");
        return;
    }

    arena_begin_run();
    run = true;
    set_time_at_start(); // for state timer (homing at 1.45min, off at 2min)

    // Serial.println("RUN,STARTED");
    Serial2.println("RUN,STARTED");

    // Serial.println("CONFIG,LOCKED");
    Serial2.println("CONFIG,LOCKED");
}

int max_i = 100;
int i = 0;

void loop()
{
    imu_update();
    xy_exe();
    tof_update();
    ultrasound_exe();
    pose_update();
    // pose_telemetry_exe();

    // Lock before processing queued commands when GO is pressed.
    checkGo();
    serial_exe();

    map_update();

    if (i > max_i) {
        // send_map_data();    
        i = 0;
    }

    i ++;

    print_proximity();


    if (!run) {
        return;
    }

    limit_switch_exe();
    colour_sensor_update();

    logic_exe();
    updateStateMachine();

    pickup_servo_exe();
    emag_exe();
    proximity_exe();

    pickup_servo_update();
    smartservo_update();
    navigator_exe();
    motor_control_update();
}
