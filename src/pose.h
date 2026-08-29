#ifndef POSE_H
#define POSE_H

#include <Arduino.h>


void pose_init();

void pose_update();

void pose_reset();

void pose_print_telemetry(Stream &port);

float pose_get_x_mm();
float pose_get_y_mm();

float pose_get_heading_deg();

void pose_telemetry_exe();
void pose_print(Stream &port);


#endif