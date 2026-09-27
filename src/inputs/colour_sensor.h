#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

bool colour_sensor_init();
bool colour_sensor_capture_home();

void colour_sensor_update();
void print_colour();

#endif