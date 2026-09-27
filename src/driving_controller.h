#ifndef DRIVING_CONTROLLER_H
#define DRIVING_CONTROLLER_H

void motor_control_init();
void motor_control_update();

void motor_control_turn_relative(float angle);
void motor_control_turn_to(float heading);

void motor_control_drive_current_heading(int basePower);
void motor_control_drive_heading(float heading, int basePower);

void motor_control_drive_to_point(
    float target_x_mm,
    float target_y_mm,
    int basePower
);

void motor_control_reverse(int power);
void motor_control_stop();

bool motor_control_is_turning();
bool motor_control_is_driving();
bool motor_control_is_driving_to_point();
bool motor_control_point_reached();

float motor_control_get_target();
float motor_control_get_error();

void motor_control_set_kp(float kp);
float motor_control_get_kp();
void motor_control_pause();

void motor_control_set_drive_kp(float kp);
float motor_control_get_drive_kp();

void motor_control_set_arrival_tolerance(float mm);
void motor_control_set_slowdown_radius(float mm);

#endif