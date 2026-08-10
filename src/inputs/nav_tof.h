#ifndef NAV_TOF_H
#define NAV_TOF_H

#include <Arduino.h>

extern uint16_t navTofDistanceMm[4];
extern bool navTofSensorOnline[4];

bool nav_tof_init();
void nav_tof_update();
void nav_tof_print(Stream &output = Serial);

#endif