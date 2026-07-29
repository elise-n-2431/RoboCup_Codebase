

// XY Sensor
// PMW3901 Optical Flow Sensor
// Works between 80mm and infinity - may require an LED

// EKF2_OF_POS_X	X position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Y	Y position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Z	Z position of optical flow focal point in body frame (default is 0.0m).
// SENS_FLOW_ROT to account for yaw

#include <SPI.h>
#include <Bitcraze_PMW3901.h>
#include "map.h"

#define FLOW_CS 10

Bitcraze_PMW3901 flow(FLOW_CS);

// 3v -> red -> 3v
// CLK -> green -> D13
// MIS -> blue -> D12
// GND -> black -> GND
// MOS -> yellow -> D11
// CS -> white -> D10
// none

int16_t og_X, og_Y;
int16_t current_X, current_Y = 0;

void xy_init()
{  
    if (!flow.begin()) {
        Serial2.println("Initialization of the flow sensor failed");
        while(1) { }
    }    
    flow.readMotionCount(&og_X, &og_Y);
}

int16_t deltaX,deltaY;

void xy_exe()
{
    flow.readMotionCount(&deltaX, &deltaY);
    current_X += deltaX;
    current_Y += deltaY;

    update_self(deltaX % 1, deltaY % 1);
    

    // Serial2.print("dX: ");
    // Serial2.print(deltaX);
    // Serial2.print(", dY: ");
    // Serial2.print(deltaY);

    Serial2.print("X: ");
    Serial2.print(current_X);
    Serial2.print(", Y: ");
    Serial2.print(current_Y);
    Serial2.print("\r\n");

    delay(1000);
}