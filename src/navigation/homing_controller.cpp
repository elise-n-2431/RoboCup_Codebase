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
#include "logic_engine.h"
#include "navigation/roaming_controller.h"
#include "navigation/reversing_controller.h"


// Tuning
static const int HOME_POWER = 430;
static const int HOME_SLOW_POWER = 340;

static const float HOME_SLOW_DISTANCE_MM = 700.0f;

static const int HOME_FRONT_BLOCK_MM = 250;
static const float HOME_AVOID_TURN_DEG = 45.0f;

static const unsigned long HOME_HEADING_UPDATE_MS = 250;

static const int HOME_DOCK_POWER = 280;
static const unsigned long HOME_DOCK_TIME_MS = 1000;
static const float HOME_DSTAR_LARGE_TURN_DEG = 50.0f;
static const float HOME_DSTAR_TURN_STEP_DEG = 30.0f;
static const float HOME_RECOVERY_REGION_MM = 1100.0f;
static const unsigned long HOME_RECOVERY_MAX_MS = 20000;
static const float HOME_RECOVERY_MAX_TRAVEL_MM = 900.0f;
static const int HOME_RECOVERY_POWER = 300;
static const int HOME_RECOVERY_FRONT_BLOCK_MM = 180;
static const int HOME_RECOVERY_WALL_NEAR_MM = 160;
static const int HOME_RECOVERY_WALL_FAR_MM = 450;
static const float HOME_RECOVERY_STEER_DEG = 15.0f;

enum HomingState
{
    HOMING_START,
    HOMING_DSTAR,
    HOMING_DSTAR_STEP_TURNING,
    HOMING_TURNING,
    HOMING_DRIVING,
    HOMING_AVOIDING,
    HOMING_RECOVERY,
    HOMING_RECOVERY_TURNING,
    HOMING_DOCKING,
    HOMING_DOCK_TURNING
};

static HomingState homingState = HOMING_START;

static unsigned long lastHomeHeadingUpdate = 0;
static unsigned long homeDockStart = 0;

static float homeDockHeading = 0.0f;
static float homeExitTargetHeading = 0.0f;
static int nearHomeFailures = 0;
static unsigned long homeRecoveryStartedAt = 0;
static float homeRecoveryStartX = 0.0f;
static float homeRecoveryStartY = 0.0f;
static float homeRecoveryTravelHeading = 0.0f;
static bool homeRecoveryWallLeft = false;
static int homeRecoveryWallBand = 99;


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

FLASHMEM static bool beginHomeRecovery(const char *reason)
{
    if (homeDistance() > HOME_RECOVERY_REGION_MM ||
        homingState == HOMING_RECOVERY ||
        homingState == HOMING_RECOVERY_TURNING)
    {
        return false;
    }

    const bool west = arena_get_home_x_mm() < ARENA_X_MM * 0.5f;
    const bool south = arena_get_home_y_mm() < ARENA_Y_MM * 0.5f;
    const float xWall = west ? 0.0f : ARENA_X_MM;
    const float yWall = south ? 0.0f : ARENA_Y_MM;
    const bool followYWall =
        fabsf(pose_get_y_mm() - yWall) <=
        fabsf(pose_get_x_mm() - xWall);

    // Travel along the nearer outer wall toward the configured corner.
    float travelX = followYWall ? (west ? -1.0f : 1.0f) : 0.0f;
    float travelY = followYWall ? 0.0f : (south ? -1.0f : 1.0f);
    float wallX = followYWall ? 0.0f : (west ? -1.0f : 1.0f);
    float wallY = followYWall ? (south ? -1.0f : 1.0f) : 0.0f;
    homeRecoveryWallLeft = (-travelY * wallX + travelX * wallY) > 0.0f;
    homeRecoveryTravelHeading = atan2f(travelY, travelX) * 180.0f / PI;
    homeRecoveryStartX = pose_get_x_mm();
    homeRecoveryStartY = pose_get_y_mm();
    homeRecoveryStartedAt = millis();
    homeRecoveryWallBand = 99;
    nearHomeFailures = 0;
    path_reset();
    motor_control_stop();
    homingState = HOMING_RECOVERY;

    debugNav.print("HOME_RECOVERY_START,");
    debugNav.print(homeRecoveryStartX);
    debugNav.print(",");
    debugNav.print(homeRecoveryStartY);
    debugNav.print(",");
    debugNav.println(reason);
    return true;
}

static bool recordHomeFailure(const char *reason)
{
    if (homeDistance() > HOME_RECOVERY_REGION_MM)
    {
        nearHomeFailures = 0;
        return false;
    }
    nearHomeFailures++;
    return nearHomeFailures >= 2 && beginHomeRecovery(reason);
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
        recordHomeFailure("NO_ROUTE");
    }
}

static void completeHomeExitAlignment()
{
    motor_control_stop();
    debugNav.print("HOME_EXIT_ALIGNMENT_COMPLETE,");
    debugNav.println(wrap180(homeExitTargetHeading - imu_get_heading()));
    resetStateFlag(&STATE_FLAGS.home_reached);
    setStateFlag(&STATE_FLAGS.home_docked);
}

void homing_on_turn_timeout()
{
    if (homing_is_docking())
    {
        if (homingState == HOMING_DOCK_TURNING &&
            STATE_FLAGS.home_reached &&
            fabsf(wrap180(homeExitTargetHeading - imu_get_heading())) <= 8.0f)
        {
            completeHomeExitAlignment();
            return;
        }
        homing_start();
        return;
    }
    if (homingState == HOMING_RECOVERY_TURNING)
    {
        debugNav.println("HOME_RECOVERY_ABORT,TURN_TIMEOUT");
        homing_start();
        return;
    }
    if (!recordHomeFailure("TURN_TIMEOUT")) homing_start();
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
        if (homingState == HOMING_RECOVERY ||
            homingState == HOMING_RECOVERY_TURNING)
        {
            debugNav.println("HOME_RECOVERY_COLOUR_FOUND");
        }
        nearHomeFailures = 0;
        path_reset();
        motor_control_stop();

        homeDockStart = millis();
        homeDockHeading = imu_get_heading();
        homingState = HOMING_DOCKING;
        motor_control_drive_heading(homeDockHeading, HOME_DOCK_POWER);
        debugNav.println("Home detected - docking");
        return;
    }

    // Keep collecting confirmed weights along the route home, including
    // after two onboard. Final approach and docking retain priority.
    if (!homing_is_docking() &&
        homeDistance() >= 350.0f &&
        roaming_check_for_weight())
    {
        path_reset();
        debugNav.print("HOMING_WEIGHT_DETECTED,");
        debugNav.println(get_weight_count());
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

            bool nearHome = homeDistance() <= HOME_RECOVERY_REGION_MM;
            if (fabsf(turn) > (nearHome ? 8.0f : 5.0f))
            {
                debugNav.print("Commanding home turn = ");
                debugNav.println(turn);
                if (nearHome) motor_control_turn_relative_homing(-turn);
                else motor_control_turn_relative(-turn);
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

            if (!path_get_lookahead_waypoint(waypointX, waypointY))
            {
                path_reset();
                motor_control_stop();

                homingState = HOMING_START;

                debugNav.println(
                    "Homing: D* route unavailable - "
                    "falling back"
                );

                recordHomeFailure("NO_ROUTE");

                break;
            }

            float dx = waypointX - pose_get_x_mm();
            float dy = waypointY - pose_get_y_mm();
            float desiredHeading = atan2f(dy, dx) * 180.0f / PI;
            float waypointHeadingError =
                wrap180(desiredHeading - pose_get_heading_deg());

            if (fabsf(waypointHeadingError) > HOME_DSTAR_LARGE_TURN_DEG)
            {
                motor_control_stop();

                float turnStep = constrain(
                    waypointHeadingError,
                    -HOME_DSTAR_TURN_STEP_DEG,
                    HOME_DSTAR_TURN_STEP_DEG
                );

                debugNav.print("HOMING_DSTAR_STEP_TURN,");
                debugNav.print(waypointHeadingError);
                debugNav.print(",");
                debugNav.println(turnStep);

                // Pose heading and IMU/motor turns have opposite signs.
                motor_control_turn_relative_coarse(-turnStep);
                homingState = HOMING_DSTAR_STEP_TURNING;
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

        case HOMING_DSTAR_STEP_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            motor_control_stop();
            debugNav.println("HOMING_DSTAR_STEP_TURN_COMPLETE");
            homingState = HOMING_DSTAR;
            return;
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

                if (homeDistance() <= HOME_RECOVERY_REGION_MM)
                    motor_control_turn_relative_homing(turnDirection);
                else
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


        case HOMING_RECOVERY_TURNING:
        {
            if (motor_control_is_turning()) return;
            motor_control_stop();
            homingState = HOMING_RECOVERY;
            break;
        }

        case HOMING_RECOVERY:
        {
            float travelX = pose_get_x_mm() - homeRecoveryStartX;
            float travelY = pose_get_y_mm() - homeRecoveryStartY;
            if (millis() - homeRecoveryStartedAt >= HOME_RECOVERY_MAX_MS ||
                sqrtf(travelX * travelX + travelY * travelY) >=
                    HOME_RECOVERY_MAX_TRAVEL_MM)
            {
                motor_control_stop();
                debugNav.println("HOME_RECOVERY_ABORT,LIMIT");
                homing_start();
                break;
            }

            int wallDistance = homeRecoveryWallLeft ? outerLeft : outerRight;
            int band = wallDistance < HOME_RECOVERY_WALL_NEAR_MM ? -1
                     : wallDistance > HOME_RECOVERY_WALL_FAR_MM ? 1 : 0;
            if (band != homeRecoveryWallBand)
            {
                homeRecoveryWallBand = band;
                debugNav.print("HOME_RECOVERY_WALL,");
                debugNav.print(homeRecoveryWallLeft ? "LEFT," : "RIGHT,");
                debugNav.println(wallDistance);
            }

            if (front < HOME_RECOVERY_FRONT_BLOCK_MM)
            {
                motor_control_stop();
                debugNav.println("HOME_RECOVERY_ABORT,FRONT_BLOCKED");
                reversing_set_reason(REVERSE_CRITICAL_OBSTACLE);
                setStateFlag(&STATE_FLAGS.reverse_triggered);
                break;
            }

            float targetPoseHeading = homeRecoveryTravelHeading +
                band * (homeRecoveryWallLeft ? 1.0f : -1.0f) *
                    HOME_RECOVERY_STEER_DEG;
            float headingError = wrap180(targetPoseHeading -
                                         pose_get_heading_deg());
            if (fabsf(headingError) > 30.0f)
            {
                motor_control_stop();
                motor_control_turn_relative_homing(
                    -constrain(headingError, -30.0f, 30.0f));
                homingState = HOMING_RECOVERY_TURNING;
                break;
            }
            if (millis() - lastHomeHeadingUpdate >= HOME_HEADING_UPDATE_MS)
            {
                lastHomeHeadingUpdate = millis();
                motor_control_drive_heading(
                    imu_get_heading() - headingError,
                    HOME_RECOVERY_POWER);
            }
            break;
        }

        case HOMING_DOCKING:
        {
            if (millis() - homeDockStart < HOME_DOCK_TIME_MS)
            {
                return;
            }

            motor_control_stop();

            // Docking heading is measured at the physical colour-confirmed
            // base. Turn back along that approach, independent of pose drift.
            homeExitTargetHeading = wrap180(homeDockHeading + 180.0f);
            float exitTurn = wrap180(homeExitTargetHeading - imu_get_heading());

            debugNav.print("Docking complete - exit turn = ");

            debugNav.println(
                exitTurn
            );

            // Pose positive rotation is CCW,
            // IMU/motor relative turn positive is clockwise.
            motor_control_turn_relative_homing(exitTurn);

            homingState = HOMING_DOCK_TURNING;

            break;
        }


        case HOMING_DOCK_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            completeHomeExitAlignment();

            break;
        }
    }
}
