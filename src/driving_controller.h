#ifndef DRIVING_CONTROLLER_H
#define DRIVING_CONTROLLER_H


void motor_control_init();

void motor_control_update();

void motor_control_turn_relative(float angle);
void motor_control_turn_to(float heading);

void motor_control_stop();

bool motor_control_is_active();

float motor_control_get_target();
float motor_control_get_error();

void motor_control_set_kp(float kp);
void motor_control_set_kd(float kd);

float motor_control_get_kp();
float motor_control_get_kd();

#endif