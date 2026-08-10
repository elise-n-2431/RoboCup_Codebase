#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H


// Initialisation
void motor_driver_init();


// Basic motion
void driveForward();
void driveReverse();
void turnLeft();
void turnRight();
void stopMotors();


// Lower-level track command
// Each argument should be:
//   +1 = forward
//    0 = stop
//   -1 = reverse
void driveTracks(int leftTrack, int rightTrack);


// Motor enable
void armMotors();
void disarmMotors();
bool motorsAreArmed();


// Speed control
void increaseDrivePower();
void decreaseDrivePower();

int getDrivePower();


#endif