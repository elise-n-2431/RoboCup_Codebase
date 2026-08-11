#ifndef TOF_EXPANDER_H
#define TOF_EXPANDER_H

void tof_init();
void tof_update();

int getToFDistance(int sensorNumber);

void tof_print_readings();


#endif