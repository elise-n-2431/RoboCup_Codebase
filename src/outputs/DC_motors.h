#ifndef DC_MOTORS_H
#define DC_MOTORS_H

enum DriveMode {
    DRIVE_SERIAL,
    DRIVE_NAVIGATOR
};

void DC_motors_init();
void DC_motors_exe(DriveMode mode);

int getDrivePower();

#endif