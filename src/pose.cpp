#include "pose.h"

#include <Arduino.h>
#include <math.h>
#include "state_machine.h"
#include "inputs/encoders.h"
#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "inputs/xy_sensor.h"
#include "arena_config.h"
#include "debug_print.h"
#include "driving_controller.h"

// ============================================================
// POSE
// ============================================================

static float poseXmm = 0.0;
static float poseYmm = 0.0;


// IMU heading when pose was reset.
// This makes position coordinates relative to the robot's
// starting direction rather than magnetic north.
static float startImuHeadingDeg = 0.0f;
static float startPoseHeadingDeg = 0.0f;


static long previousLeftCount = 0;
static long previousRightCount = 0;

static const unsigned long TELEMETRY_PERIOD_MS = 100;
static unsigned long lastTelemetryTime = 0;
static bool telemetryEnabled = true;

static const float XY_FUSION_WEIGHT = 0.00f;

void pose_init()
{
    pose_reset();
}


void pose_reset()
{
    const ArenaConfig& config = arena_get_config();

    poseXmm = config.startX;
    poseYmm = config.startY;

    startImuHeadingDeg = imu_get_heading();

    startPoseHeadingDeg = config.startHeading;

    previousLeftCount = encoders_get_left_count();
    previousRightCount = encoders_get_right_count();

    // float unusedForward, unusedLateral;
    // get_xy_delta_mm(unusedForward, unusedLateral);
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

    float leftDistance =
        deltaLeftCount * encoders_get_left_mm_per_count();

    float rightDistance =
        deltaRightCount * encoders_get_right_mm_per_count();

    float encoderForward =
        (leftDistance + rightDistance) / 2.0f;

    // Only accept encoder movement if the IMU detects
    // actual translational movement.
    float forwardDistance =
    encoderForward;

    // Encoder mismatch during an intentional point turn is
    // rotation, not useful translational motion.
    if (motor_control_is_turning())
    {
        forwardDistance = 0.0f;
    }

    // Suppress impossible forward odometry while physically
    // pushing something directly in front of the robot.
    if (forwardDistance > 0.0f &&
        (getNavState() == ROAMING ||
        getNavState() == HOMING) &&
        (motor_control_is_driving() ||
        motor_control_is_driving_to_point()))
    {
        int frontContact = 1200;

        int distances[4] =
        {
            tof_get_nav_outer_left(),
            tof_get_nav_inner_left(),
            tof_get_nav_inner_right(),
            tof_get_nav_outer_right()
        };

        for (int i = 0; i < 4; i++)
        {
            if (distances[i] > 0 &&
                distances[i] < frontContact)
            {
                frontContact =
                    distances[i];
            }
        }

        if (frontContact < 30)
        {
            forwardDistance = 0.0f;

            debugPose.println(
                "POSE_FORWARD_SUPPRESSED_CONTACT"
            );
        }
    }
    float headingDeg = pose_get_heading_deg();
    float headingRad = headingDeg * PI / 180.0f;

    poseXmm +=
        forwardDistance * cos(headingRad);

    poseYmm +=
        forwardDistance * sin(headingRad);
}

void pose_apply_correction(float dx_mm, float dy_mm)
{
    poseXmm += dx_mm;
    poseYmm += dy_mm;

    Serial.print("dx");
    Serial.println(dx_mm);

    Serial.print("dy");
    Serial.println(dy_mm);
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
    float imuDelta = imu_get_heading() - startImuHeadingDeg;

    while (imuDelta > 180.0f)
    {
        imuDelta -= 360.0f;
    }

    while (imuDelta < -180.0f)
    {
        imuDelta += 360.0f;
    }

    // IMU positive rotation is clockwise.
    // Arena/pose positive rotation is counter-clockwise.
    float heading = startPoseHeadingDeg - imuDelta;

    while (heading >= 360.0f)
    {
        heading -= 360.0f;
    }

    while (heading < 0.0f)
    {
        heading += 360.0f;
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

    // pose_print_telemetry(Serial);
    // pose_print_telemetry(Serial2);
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


