#include "pose.h"

#include <Arduino.h>
#include <math.h>
#include "state_machine.h"
#include "inputs/encoders.h"
#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "inputs/xy_sensor.h"


// ============================================================
// POSE
// ============================================================

static float poseXmm = 0.0;
static float poseYmm = 0.0;


// IMU heading when pose was reset.
// This makes position coordinates relative to the robot's
// starting direction rather than magnetic north.
static float startHeadingDeg = 0.0;


static long previousLeftCount = 0;
static long previousRightCount = 0;

static const unsigned long TELEMETRY_PERIOD_MS = 100;
static unsigned long lastTelemetryTime = 0;
static bool telemetryEnabled = true;

static const float XY_FUSION_WEIGHT = 0.0f;

void pose_init()
{
    pose_reset();
}


void pose_reset()
{
    poseXmm = 300.0;
    poseYmm = 300.0;


    startHeadingDeg =
        imu_get_heading();


    previousLeftCount =
        encoders_get_left_count();

    previousRightCount =
        encoders_get_right_count();

}


// ============================================================
// UPDATE
// ============================================================

void pose_update()
{
    if (!encoders_is_calibrated())
    {
        return;
    }

    long leftCount = encoders_get_left_count();
    long rightCount = encoders_get_right_count();

    long deltaLeftCount = leftCount - previousLeftCount;
    long deltaRightCount = rightCount - previousRightCount;

    previousLeftCount = leftCount;
    previousRightCount = rightCount;

    float leftDistance = deltaLeftCount * encoders_get_left_mm_per_count();
    float rightDistance = deltaRightCount * encoders_get_right_mm_per_count();

    float encoderForward = (leftDistance + rightDistance) / 2.0f;


    // Body-frame delta since the last pose_update() -- drains the
    // xy_sensor accumulator.
    float xyForward, xyLateral;
    get_xy_delta_mm(xyForward, xyLateral);


    float forwardDistance =
        (1.0f - XY_FUSION_WEIGHT) * encoderForward
        + XY_FUSION_WEIGHT * xyForward;

    float lateralDistance = XY_FUSION_WEIGHT * xyLateral;


    float headingDeg = imu_get_heading() - startHeadingDeg;

    while (headingDeg >= 360.0f) headingDeg -= 360.0f;
    while (headingDeg < 0.0f)    headingDeg += 360.0f;

    float headingRad = headingDeg * PI / 180.0f;

    // Serial.print("forward: ");
    // Serial.println(forwardDistance);
    // Serial.print(", lateral: ");
    // Serial.println(lateralDistance);
    // pose_print(Serial);

    poseXmm += forwardDistance * cos(headingRad) - lateralDistance * sin(headingRad);
    poseYmm += forwardDistance * sin(headingRad) + lateralDistance * cos(headingRad);

    static unsigned long lastPoseDebug = 0;

    if (millis() - lastPoseDebug >= 250)
    {
        lastPoseDebug = millis();

        Serial2.print("POSE ENC: dL=");
        Serial2.print(deltaLeftCount);

        Serial2.print(" dR=");
        Serial2.print(deltaRightCount);

        Serial2.print(" leftMM=");
        Serial2.print(leftDistance);

        Serial2.print(" rightMM=");
        Serial2.print(rightDistance);

        Serial2.print(" forward=");
        Serial2.println(encoderForward);
    }

}


// ============================================================
// GETTERS
// ============================================================

float pose_get_x_mm()
{
    return poseXmm;
}


float pose_get_y_mm()
{
    return poseYmm;
}


float pose_get_heading_deg()
{
    float heading =
        imu_get_heading()
        - startHeadingDeg;


    while (heading >= 360.0)
    {
        heading -= 360.0;
    }


    while (heading < 0.0)
    {
        heading += 360.0;
    }


    return heading;
}


// ============================================================
// DEBUG
// ============================================================

void pose_print(Stream &port)
{
    port.print("POSE X: ");
    port.print(poseXmm);

    port.print(" mm   Y: ");
    port.print(poseYmm);

    port.print(" mm   H: ");
    port.print(
        pose_get_heading_deg()
    );

    port.println(" deg");
}



void pose_telemetry_exe()
{
    if (millis() - lastTelemetryTime < TELEMETRY_PERIOD_MS) return;

    lastTelemetryTime = millis();

    pose_print_telemetry(Serial);
    pose_print_telemetry(Serial2);
}

void pose_print_telemetry(Stream &port)
{
    port.print("ROBOT,");

    port.print(pose_get_x_mm());
    port.print(",");

    port.print(pose_get_y_mm());
    port.print(",");

    port.print(pose_get_heading_deg());
    port.print(",");

    port.print(getNavStateName());
    port.print(",");

    port.print(getCollectStateName());
    port.print(",");

    port.print(tof_get_nav_outer_left());
    port.print(",");

    port.print(tof_get_nav_inner_left());
    port.print(",");

    port.print(tof_get_nav_inner_right());
    port.print(",");

    port.print(tof_get_nav_outer_right());
    port.print(",");

    port.print(tof_get_weight_left_top());
    port.print(",");

    port.print(tof_get_weight_left_bottom());
    port.print(",");

    port.print(tof_get_weight_right_top());
    port.print(",");

    port.print(tof_get_weight_right_bottom());
    port.print(",");

    port.println(tof_get_weight_middle());
}


