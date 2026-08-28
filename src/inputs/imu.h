#ifndef IMU_H
#define IMU_H

// Initialise the BNO055.
// Returns true if the sensor was found.
bool imu_init();


// Update heading, pitch, roll and calibration status.
void imu_update();


// Current orientation values in degrees.
float imu_get_heading();
float imu_get_pitch();
float imu_get_roll();


// Returns the four BNO055 calibration scores.
// Each score ranges from 0 to 3.
void imu_get_calibration(
    int &system,
    int &gyro,
    int &accel,
    int &mag
);


// Returns true if the IMU successfully initialised.
bool imu_is_online();


// Debug printing.
void imu_print_readings();


// Remove saved calibration from EEPROM.
// Restart the robot afterwards and recalibrate.
void imu_clear_calibration();

#endif