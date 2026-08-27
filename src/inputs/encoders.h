#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>


bool encoders_init();

long encoders_get_left_count();
long encoders_get_right_count();

void encoders_reset();


void encoders_set_mm_per_count(
    float leftMmPerCount,
    float rightMmPerCount
);

bool encoders_is_calibrated();

float encoders_get_left_mm_per_count();
float encoders_get_right_mm_per_count();

float encoders_get_left_distance_mm();
float encoders_get_right_distance_mm();


void encoders_print(Stream &port);


#endif
