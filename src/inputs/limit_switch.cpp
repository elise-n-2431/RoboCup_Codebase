#include "state_machine.h"
#include "limit_switch.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1

const byte SX1509_LIMIT_ADDRESS = 0x3E;
const byte AIO6_PIN = 6;
SX1509 io;

const int CONSECUTIVE_HITS = 50;
int count_switch_on = 0;
int count_switch_off = 0;


void limit_switch_init() {
  io.pinMode(AIO6_PIN, INPUT_PULLUP);

}

void limit_switch_exe() {
    if (getCollectState() != VERT_REACHED &&
        getCollectState() != HORI_REACHED) {
        count_switch_on = 0;
        count_switch_off = 0;
        return;
    }

    if (io.digitalRead(AIO6_PIN) == LOW) {
        count_switch_on++;
        count_switch_off = 0;
    } else {
        count_switch_on = 0;
        count_switch_off++;
    }

    if (count_switch_on >= 50) {
        setStateFlag(&STATE_FLAGS.magnet_hit);
        count_switch_on = 0;
        count_switch_off = 0;
    }
    else if (count_switch_off >= 50) {
        if (getCollectState() == VERT_REACHED) {
            setStateFlag(&STATE_FLAGS.no_vertical);
        }
        else if (getCollectState() == HORI_REACHED) {
            setStateFlag(&STATE_FLAGS.no_horizontal);
        }

        count_switch_on = 0;
        count_switch_off = 0;
    }
}

bool getLimitSwitch() {
    return io.digitalRead(AIO6_PIN) == LOW
}