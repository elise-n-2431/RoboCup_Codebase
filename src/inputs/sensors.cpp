#include "sensors.h"

SensorFlags SENSOR_FLAGS;

void setSensorFlag(bool* flag) {
    *flag = true;
}

void resetSensorFlag(bool* flag) {
    *flag = false;
}