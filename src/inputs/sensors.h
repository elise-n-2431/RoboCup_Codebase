//
// Created by elise on 21/07/2026.
//

#ifndef SENSORS_H
#define SENSORS_H

struct SensorFlags {
    // sensor flags trigger a decision by the logic machine, or peripheral behavior (non state changing events)
    bool metal_detected = false;
    bool limit_switch_on = false;

};

void setSensorFlag(bool*);
void resetSensorFlag(bool*);

extern SensorFlags SENSOR_FLAGS;

#endif