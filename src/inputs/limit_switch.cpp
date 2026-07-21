#include "state_machine.h"
#include "sensors.h"
#include "limit_switch.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1

const byte SX1509_LIMIT_ADDRESS = 0x3E;
const byte AIO6_PIN = 6;
SX1509 io;

void limit_switch_init() {
  io.pinMode(AIO6_PIN, INPUT_PULLUP);

}

void limit_switch_exe() {
    if (io.digitalRead(AIO6_PIN) == LOW) {
        setSensorFlag(&SENSOR_FLAGS.limit_switch_on);
    } else {
        resetSensorFlag(&SENSOR_FLAGS.limit_switch_on)
    }
}