#include "homing_controller.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "pose.h"
#include "arena_config.h"
#include "debug_print.h"
#include "path_finding.h"


// Tuning
static const int HOME_POWER = 430;
static const int HOME_SLOW_POWER = 340;

static const float HOME_SLOW_DISTANCE_MM = 700.0f;

static const int HOME_FRONT_BLOCK_MM = 250;
static const float HOME_AVOID_TURN_DEG = 45.0f;

static const unsigned long HOME_HEADING_UPDATE_MS = 250;

static const int HOME_DOCK_POWER = 280;
static const unsigned long HOME_DOCK_TIME_MS = 1000;

enum HomingState
{
    HOMING_START,
    HOMING_DSTAR,
    HOMING_TURNING,
    HOMING_DRIVING,
    HOMING_AVOIDING,
    HOMING_DOCKING,
    HOMING_DOCK_TURNING
};

static HomingState homingState = HOMING_START;

static unsigned long lastHomeHeadingUpdate = 0;
static unsigned long homeDockStart = 0;

static float homeDockHeading = 0.0f;


static float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    return angle;
}


static int homeClearanceValue(int distance)
{
    // 0 means no obstacle return, so treat as clear space.
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}

static float homeDistance()
{
    float dx = arena_get_home_x_mm() - pose_get_x_mm();
    float dy = arena_get_home_y_mm() - pose_get_y_mm();

    return sqrtf(dx * dx + dy * dy);
}

static float homeHeadingError()
{
    float dx = arena_get_home_x_mm() - pose_get_x_mm();
    float dy = arena_get_home_y_mm() - pose_get_y_mm();

    float desiredPoseHeading = atan2f(dy, dx) * 180.0f / PI;
    float currentPoseHeading = pose_get_heading_deg();

    return wrap180(desiredPoseHeading - currentPoseHeading);
}

void homing_start()
{
    homingState = HOMING_START;

    lastHomeHeadingUpdate = 0;
    homeDockStart = 0;

    debugNav.println(
        "Navigator: HOMING started"
    );

    if (path_init())
    {
        homingState = HOMING_DSTAR;

        debugNav.println("Homing: D* route ready");
    }
    else
    {
        path_reset();

        debugNav.println("Homing: no D* route - ""using existing homing");
    }
}

bool homing_is_docking()
{
    return homingState == HOMING_DOCKING ||
           homingState == HOMING_DOCK_TURNING;
}

void homing_update()
{
    // Colour sensor has final authority over reaching home.
    if (STATE_FLAGS.home_reached && !homing_is_docking())
    {
        path_reset();
        motor_control_stop();

        homeDockStart = millis();
        homeDockHeading = imu_get_heading();
        homingState = HOMING_DOCKING;
        motor_control_drive_heading(homeDockHeading, HOME_DOCK_POWER);
        debugNav.println("Home detected - docking");
        return;
    }


    int outerLeft = homeClearanceValue(tof_get_nav_outer_left());
    int innerLeft = homeClearanceValue(tof_get_nav_inner_left());
    int innerRight = homeClearanceValue(tof_get_nav_inner_right());
    int outerRight = homeClearanceValue(tof_get_nav_outer_right());

    int front = min(innerLeft, innerRight);
    int leftClearance = min(outerLeft, innerLeft);
    int rightClearance = min(outerRight, innerRight);


    switch (homingState)
    {
        case HOMING_START:
        {
            motor_control_stop();
            debugNav.println("----- HOMING START -----");
            debugNav.print("Pose X = ");
            debugNav.println(pose_get_x_mm());
            debugNav.print("Pose Y = ");
            debugNav.println(pose_get_y_mm());
            debugNav.print("Pose heading = ");
            debugNav.println(pose_get_heading_deg());
            debugNav.print("IMU heading = ");
            debugNav.println(imu_get_heading());

            float dx = arena_get_home_x_mm() - pose_get_x_mm();
            float dy = arena_get_home_y_mm() - pose_get_y_mm();

            debugNav.print("dx home = ");
            debugNav.println(dx);

            debugNav.print("dy home = ");
            debugNav.println(dy);

            float desiredPoseHeading = atan2f(dy, dx) * 180.0f / PI;

            debugNav.print("Desired pose heading = ");
            debugNav.println(desiredPoseHeading);


            float turn = homeHeadingError();

            debugNav.print("Homing relative turn = ");
            debugNav.println(turn);

            if (fabsf(turn) > 5.0f)
            {
                debugNav.print("Commanding home turn = ");
                debugNav.println(turn);
                motor_control_turn_relative(-turn);
                homingState = HOMING_TURNING;
            }
            else
            {
                homingState = HOMING_DRIVING;
            }

            break;
        }

        case HOMING_DSTAR:
        {
            // Once close to home, hand back to the
            // already-tested direct homing + colour docking.
            if (homeDistance() < 350.0f)
            {
                path_reset();
                motor_control_stop();

                homingState = HOMING_START;

                debugNav.println(
                    "Homing: D* near home - "
                    "switching to final approach"
                );

                break;
            }

            float waypointX;
            float waypointY;

            if (!path_get_next_waypoint(waypointX, waypointY))
            {
                path_reset();
                motor_control_stop();

                homingState = HOMING_START;

                debugNav.println(
                    "Homing: D* route unavailable - "
                    "falling back"
                );

                break;
            }

            int power =
                homeDistance() <
                HOME_SLOW_DISTANCE_MM
                ? HOME_SLOW_POWER
                : HOME_POWER;

            motor_control_drive_to_point(
                waypointX,
                waypointY,
                power
            );

            break;
        }

        case HOMING_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            lastHomeHeadingUpdate = 0;
            homingState = HOMING_DRIVING;

            break;
        }


        case HOMING_DRIVING:
        {
            // Basic obstacle avoidance until path planning is integrated.
            if (front < HOME_FRONT_BLOCK_MM)
            {
                motor_control_stop();

                float turnDirection =
                    leftClearance > rightClearance
                    ? -HOME_AVOID_TURN_DEG
                    : HOME_AVOID_TURN_DEG;

                debugNav.println("Homing: obstacle avoidance");

                motor_control_turn_relative(turnDirection);

                homingState = HOMING_AVOIDING;

                return;
            }


            // Periodically update the heading toward home.
            if (millis() - lastHomeHeadingUpdate >= HOME_HEADING_UPDATE_MS)
            {
                lastHomeHeadingUpdate = millis();

                float relativeError = homeHeadingError();

                // homeHeadingError() is in pose coordinates.
                // Convert that relative correction to an absolute IMU heading.
                float targetHeading = imu_get_heading() - relativeError;

                int power =
                    homeDistance() < HOME_SLOW_DISTANCE_MM
                    ? HOME_SLOW_POWER
                    : HOME_POWER;

                motor_control_drive_heading(targetHeading, power);
            }

            break;
        }


        case HOMING_AVOIDING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            // Recalculate the route toward home after the avoidance turn.
            homingState = HOMING_START;

            break;
        }


        case HOMING_DOCKING:
        {
            if (millis() - homeDockStart < HOME_DOCK_TIME_MS)
            {
                return;
            }

            motor_control_stop();

            debugNav.println("Docking complete - turning 180");

            motor_control_turn_relative(180.0f);

            homingState = HOMING_DOCK_TURNING;

            break;
        }


        case HOMING_DOCK_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            debugNav.println("Home 180 turn complete");

            resetStateFlag(&STATE_FLAGS.home_reached);
            setStateFlag(&STATE_FLAGS.home_docked);

            break;
        }
    }
}