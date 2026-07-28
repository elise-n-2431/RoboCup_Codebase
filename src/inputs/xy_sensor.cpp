

// XY Sensor
// PMW3901 Optical Flow Sensor
// Works between 80mm and infinity - may require an LED

// EKF2_OF_POS_X	X position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Y	Y position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Z	Z position of optical flow focal point in body frame (default is 0.0m).
// SENS_FLOW_ROT to account for yaw

#include <SPI.h>
#include <Bitcraze_PMW3901.h>

#define FLOW_CS 10

Bitcraze_PMW3901 flow(FLOW_CS);


void xy_init()
{
    Serial.println("Starting PMW3901 init");

    Serial.begin(115200);
    Serial2.begin(115200);

    SPI.begin();

    Serial.println("SPI started");

    if (!flow.begin()) {
        Serial.println("PMW3901 failed");
        Serial2.println("FLOW ERROR");

        while(1);
    }

    Serial.println("PMW3901 ready");
}

void xy_exe()
{
    int16_t x, y;

    flow.readMotionCount(&x, &y);

    Serial.print("X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.println(y);

    Serial2.print("FLOW:");
    Serial2.print(x);
    Serial2.print(",");
    Serial2.println(y);
}