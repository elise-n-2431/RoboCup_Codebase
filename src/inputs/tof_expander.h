#ifndef TOF_EXPANDER_H
#define TOF_EXPANDER_H

void tof_init();
void tof_update();

int tof_get_distance(int sensorNumber);

// Navigation ToFs
int tof_get_nav_outer_left();
int tof_get_nav_inner_left();
int tof_get_nav_inner_right();
int tof_get_nav_outer_right();


// Weight detection ToFs
int tof_get_weight_left_top();
int tof_get_weight_left_bottom();
int tof_get_weight_right_top();
int tof_get_weight_right_bottom();


void tof_print_readings(Stream &port);


#endif