#ifndef DC_MOTORS_H
#define DC_MOTORS_H

enum DriveMode {
    DRIVE_SERIAL,
    DRIVE_NAVIGATOR,
};

void DC_motors_init();
void DC_motors_exe(DriveMode mode);

//allows for PD control so doesnt just have to send -1 and 1 to the motors.
void DC_motors_setPower(int leftPower, int rightPower);


int getDrivePower();

#endif