#ifndef SMART_SERVO_H
#define SMART_SERVO_H

#include <Arduino.h>


bool smartservo_init(
    uint8_t servoId
);

void smartservo_update();

bool smartservo_ping();

void smartservo_torque_on();

void smartservo_torque_off();

void smartservo_set_position(
    uint16_t position,
    uint8_t playtime
);

uint16_t smartservo_get_position();

void smartservo_print_status();

void smartservo_scan();

void smartservo_gate_open();

void smartservo_gate_close();


#endif