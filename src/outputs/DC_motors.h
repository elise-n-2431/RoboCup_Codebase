#ifndef DC_MOTORS_H
#define DC_MOTORS_H

#include "comms/serial.h"

void DC_motors_init();

void DC_motors_exe(RobotCommand command);

//allows for PD control so doesnt just have to send -1 and 1 to the motors.
void DC_motors_setPower(int leftPower, int rightPower);


int getDrivePower();

#endif