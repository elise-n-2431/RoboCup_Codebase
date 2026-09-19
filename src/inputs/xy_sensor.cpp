

// XY Sensor
// PMW3901 Optical Flow Sensor
// Works between 80mm and infinity - may require an LED

// EKF2_OF_POS_X	X position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Y	Y position of optical flow focal point in body frame (default is 0.0m).
// EKF2_OF_POS_Z	Z position of optical flow focal point in body frame (default is 0.0m).
// SENS_FLOW_ROT to account for yaw

#include <SPI.h>
#include <Bitcraze_PMW3901.h>
// #include "map.h"

#define FLOW_CS 10

Bitcraze_PMW3901 flow(FLOW_CS);

// 3v -> red -> 3v
// CLK -> green -> D13
// MIS -> blue -> D12
// GND -> black -> GND
// MOS -> yellow -> D11
// CS -> white -> D10
// none


const float SENSOR_HEIGHT_MM =225.0; // estimate (UPDATE)
const float MM_PER_PIXEL = 0.30;

int16_t og_X, og_Y;

static float pendingDeltaXmm = 0.0f;
static float pendingDeltaYmm = 0.0f;

int16_t deltaX, deltaY;

// Note: mm_moved = counts × (height_mm / focal_constant)

// int get_xy_x_mm() {
//     return current_X_mm;
// }

// int get_xy_y_mm() {
//     return current_Y_mm;
// }

void get_xy_delta_mm(float &deltaXmm_out, float &deltaYmm_out)
{
    deltaXmm_out = pendingDeltaXmm;
    deltaYmm_out = pendingDeltaYmm;

    pendingDeltaXmm = 0.0f;
    pendingDeltaYmm = 0.0f;
}

void print_xy()
{
    Serial.print("pending dX_mm: ");
    Serial.print(pendingDeltaXmm);
    Serial.print(", dY_mm: ");
    Serial.println(pendingDeltaYmm);
}

void xy_init()
{  
    if (!flow.begin()) {
        Serial.println("Initialization of the flow sensor failed");
        while(1) { }
    }    
    flow.readMotionCount(&og_X, &og_Y);
}

void xy_exe()
{
    flow.readMotionCount(&deltaX, &deltaY);

    pendingDeltaXmm += deltaX * MM_PER_PIXEL;
    pendingDeltaYmm += deltaY * MM_PER_PIXEL;
}
