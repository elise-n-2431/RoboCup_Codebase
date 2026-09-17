#ifndef SMART_SERVO_H
#define SMART_SERVO_H

#include <Arduino.h>


bool smartservo_init();

void smartservo_update();

bool smartservo_ping();

void smartservo_torque_on();

void smartservo_torque_off();

void smartservo_set_position(
    uint8_t servoId,
    uint16_t position,
    uint8_t playtime
);

uint16_t smartservo_get_position(
    uint8_t servoId
);

void smartservo_print_status(
    uint8_t servoId
);

void smartservo_scan();

void smartservo_arms_open();
void smartservo_arms_close();


void smartservo_test_left(uint16_t position);
void smartservo_test_right(uint16_t position);

void smartservo_print_positions();


#endif