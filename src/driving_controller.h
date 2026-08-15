#ifndef DRIVING_CONTROLLER_H
#define DRIVING_CONTROLLER_H


void motor_control_init();

void motor_control_update();

void motor_control_turn_relative(float angle);
void motor_control_turn_to(float heading);

//try drive straight
void motor_control_drive_current_heading(int basePower);

void motor_control_drive_heading(float heading, int basePower);


void motor_control_stop();

bool motor_control_is_active();

float motor_control_get_target();
float motor_control_get_error();

void motor_control_set_target_heading(float heading);

void motor_control_set_kp(float kp);
void motor_control_set_kd(float kd);

float motor_control_get_kp();
float motor_control_get_drive_kp();
void motor_control_set_drive_kp(float kp);

bool motor_control_is_turning();
bool motor_control_is_driving();


#endif