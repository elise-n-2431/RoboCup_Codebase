#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Arduino.h>

void ultrasound_init();
void ultrasound_exe();

float ultrasound_get_left_mm();
float ultrasound_get_right_mm();

#endif