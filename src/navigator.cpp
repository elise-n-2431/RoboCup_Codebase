#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "inputs/tof_expander.h"
#include "state_machine.h"
#include "outputs/smart_servo.h"
#include "map.h"
#include "pose.h"
#include "arena_config.h"
#include "debug_print.h"
#include "priority_targets.h"
#include "navigation/weight_detection.h"
#include "navigation/pursuit_controller.h"
#include "navigation/reversing_controller.h"
#include "navigation/homing_controller.h"


enum RoamingState
{
    ROAM_START,
    ROAM_DRIVING,
    ROAM_TURNING,
    ROAM_CHECKING,

    // Turning specifically toward a known priority target.
    ROAM_TARGET_TURNING,

    // At the expected location, stationary while the real
    // weight sensors get a chance to confirm something exists.
    ROAM_TARGET_WAITING
};

static RoamingState roamingState = ROAM_START;


// ============================================================
// ROAMING TUNING
// ============================================================

static int ROAM_POWER = 430;

// Distance at which normal avoidance begins
static int ROAM_FRONT_BLOCK_MM = 250;

static int ROAM_CRITICAL_MM = 90;

// Normal avoidance turn
static float ROAM_AVOID_TURN_DEG = 45.0f;


static const int ROAM_SIDE_BLOCK_MM = 180;
static const int ROAM_SLOW_MM = 500;
static const int ROAM_SLOW_POWER = 340;


static const unsigned long NAV_TURN_TIMEOUT_MS = 8000;

static bool navigatorEnabled = true;
static bool roamingPickupEnabled = false;
static bool turnWatchActive = false;

static NavState lastNavState = STATIONARY;

static unsigned long navigatorStateStarted = 0;
static unsigned long turnStartedAt = 0;
static unsigned long roamCheckStartedAt = 0;
static unsigned long lastWeightCheckAt = 0;

static float roamHeading = 0.0f;

static float roamCommandedHeading = NAN;

static int roamCommandedPower = 0;

static int roamTurnDirection = 1;


// ============================================================
// PRIORITY TARGET ROAMING
// ============================================================

// We don't try to drive the centre of the robot directly
// onto the weight coordinate.
//
// At this distance, stop and let the real weight sensors
// decide whether something actually exists there.
static const float PRIORITY_TARGET_ARRIVAL_MM = 250.0f;

// If the desired target is substantially off our current
// heading, point-turn before driving toward it.
static const float PRIORITY_TARGET_TURN_THRESHOLD_DEG =
    18.0f;
// When at the expected target position, align a bit more
// accurately before waiting for weight detection.
static const float PRIORITY_TARGET_FINAL_ALIGN_DEG =
    8.0f;
// Time to wait at an expected location before deciding
// the weight isn't actually there.
static const unsigned long PRIORITY_TARGET_CONFIRM_MS =
    1200;


// IMPORTANT:
//
// After avoiding an obstacle, don't instantly aim directly
// back toward the target. Drive along the escape heading
// briefly so we actually get around the obstacle.
//
// This prevents the same kind of loop currently possible
// in HOMING.
static const unsigned long PRIORITY_TARGET_REJOIN_DELAY_MS = 700;
static unsigned long priorityTargetWaitStartedAt = 0;
static unsigned long priorityRejoinAllowedAt = 0;




static float wrap180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static bool getPriorityTargetInfo(
    float &targetX,
    float &targetY,
    float &distance,
    float &headingError)
{
    // Priority positions only matter during AUTO.
    //
    // Plain ROAM keeps the old behaviour.
    if (!roamingPickupEnabled)
    {
        return false;
    }


    if (priority_targets_count() == 0)
    {
        return false;
    }


    // For now the first item in the list is the highest
    // priority target.
    if (!priority_targets_get(
            0,
            targetX,
            targetY))
    {
        return false;
    }


    float dx =
        targetX -
        pose_get_x_mm();

    float dy =
        targetY -
        pose_get_y_mm();


    distance =
        sqrtf(
            dx * dx +
            dy * dy
        );


    // Target heading is calculated in the same arena/pose
    // coordinate system as the position estimate.
    float desiredPoseHeading =
        atan2f(
            dy,
            dx
        ) *
        180.0f / PI;


    headingError =
        wrap180(
            desiredPoseHeading -
            pose_get_heading_deg()
        );


    return true;
}




void navigator_init()
{
    navigator_stop();
}


static int clearanceValue(int distance)
{
    // 0 currently means nothing useful was detected,
    // so treat it as far away for choosing the clearer side.
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}


void navigator_stop()
{
    navigatorEnabled = false;
    motor_control_stop();

    turnWatchActive = false;
    roamingState = ROAM_START;
    weight_detection_reset();
    if (getNavState() == ROAMING) {
        resetStateFlag(&STATE_FLAGS.target_identified);
    }

    if (getNavState() == PURSUIT) {
        setStateFlag(&STATE_FLAGS.target_lost);
    }

    if (getNavState() == REVERSING) {
        setStateFlag(&STATE_FLAGS.reverse_complete);
    }
}

bool navigator_start(bool enablePickup)
{
    if (getCollectState() != IDLE ||
        (getNavState() != ROAMING && getNavState() != STATIONARY)) {
        // debugNav.println("Navigator: start from ROAMING or STATIONARY");
        return false;
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) return false;

    float flatPitchReference = imu_get_pitch();
    reversing_set_flat_pitch_reference(flatPitchReference);

    debugNav.print("Navigator: flat pitch reference = ");
    debugNav.println(flatPitchReference);
    navigator_stop();

    navigatorEnabled = true;
    roamingPickupEnabled = enablePickup;
    debugNav.print(
    "Navigator: priority targets loaded = "
    );
    debugNav.println(
        priority_targets_count()
    );

    priority_targets_print(
        Serial
    );

    priority_targets_print(
        Serial2
    );
    lastNavState = getNavState();

    navigatorStateStarted = millis();
    lastWeightCheckAt = millis();
    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    priorityTargetWaitStartedAt = 0;
    priorityRejoinAllowedAt = 0;
    pursuit_reset();

    // // debugNav.println(enablePickup ? "Navigator: roaming + pickup" : "Navigator: roaming only");
    return true;
}

static void roaming_start_turn(
    int leftClearance,
    int rightClearance)
{
    if (abs(
            leftClearance -
            rightClearance) > 80)
    {
        roamTurnDirection =
            leftClearance >
                    rightClearance
                ? -1
                : 1;
    }


    motor_control_stop();


    motor_control_turn_relative(
        roamTurnDirection *
        ROAM_AVOID_TURN_DEG
    );


    roamingState =
        ROAM_TURNING;


    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    priorityTargetWaitStartedAt = 0;

    weight_detection_reset_side_evidence();

    debugNav.println(
        roamTurnDirection < 0
            ? "Roaming: turn LEFT"
            : "Roaming: turn RIGHT"
    );
}

static void roaming_drive(int power)
{
    bool sameHeading =
        isfinite(
            roamCommandedHeading
        ) &&
        fabsf(
            wrap180(
                roamHeading -
                roamCommandedHeading
            )
        ) < 2.0f;


    if (motor_control_is_driving() &&
        roamCommandedPower == power &&
        sameHeading)
    {
        return;
    }


    motor_control_drive_heading(
        roamHeading,
        power
    );


    roamCommandedPower =
        power;

    roamCommandedHeading =
        roamHeading;
}

static void roaming_exe()
{
    // ========================================================
    // NORMAL WEIGHT DETECTION STILL HAS FIRST PRIORITY
    // ========================================================

    if (roamingPickupEnabled &&
        millis() -
                navigatorStateStarted >=
            1000 &&
        millis() -
                lastWeightCheckAt >=
            100)
    {
        lastWeightCheckAt =
            millis();

        WeightTargetSide detectedTarget = weight_detection_update();
        if (detectedTarget != TARGET_NONE)
        {
            pursuit_start(detectedTarget);

            motor_control_stop();
            setStateFlag(&STATE_FLAGS.target_identified);

            return;
        }
    }


    // ========================================================
    // CURRENT PRIORITY TARGET
    // ========================================================

    float priorityX = 0.0f;
    float priorityY = 0.0f;

    float priorityDistance = 0.0f;
    float priorityHeadingError = 0.0f;


    bool hasPriorityTarget =
        getPriorityTargetInfo(
            priorityX,
            priorityY,
            priorityDistance,
            priorityHeadingError
        );


    // ========================================================
    // FINISH TARGET-DIRECTION TURN
    // ========================================================

    if (roamingState ==
        ROAM_TARGET_TURNING)
    {
        if (motor_control_is_turning())
        {
            return;
        }


        motor_control_stop();

        roamCommandedPower = 0;
        roamCommandedHeading = NAN;


        roamCheckStartedAt =
            millis();

        roamingState =
            ROAM_CHECKING;

        return;
    }


    // ========================================================
    // WAIT AT EXPECTED TARGET POSITION
    // ========================================================

    if (roamingState ==
        ROAM_TARGET_WAITING)
    {
        motor_control_stop();


        // Target may have been removed elsewhere, for example
        // after a successful pickup.
        if (!hasPriorityTarget)
        {
            priorityTargetWaitStartedAt = 0;

            roamingState =
                ROAM_START;

            return;
        }


        if (priorityTargetWaitStartedAt == 0)
        {
            priorityTargetWaitStartedAt =
                millis();
        }


        // Weight detection continues at the top of this
        // function while we're waiting.
        //
        // If no real target has appeared after this period,
        // assume this expected location is empty.
        if (millis() -
                priorityTargetWaitStartedAt >=
            PRIORITY_TARGET_CONFIRM_MS)
        {
            debugNav.print(
                "Priority target empty: removing ("
            );

            debugNav.print(
                priorityX
            );

            debugNav.print(",");
            debugNav.print(
                priorityY
            );

            debugNav.println(")");


            priority_targets_remove(0);


            priorityTargetWaitStartedAt = 0;

            roamCommandedPower = 0;
            roamCommandedHeading = NAN;

            roamingState =
                ROAM_START;

            return;
        }


        return;
    }


    // ========================================================
    // ARRIVED NEAR EXPECTED WEIGHT LOCATION
    // ========================================================

    if (hasPriorityTarget &&
        priorityDistance <=
            PRIORITY_TARGET_ARRIVAL_MM)
    {
        motor_control_stop();

        roamCommandedPower = 0;
        roamCommandedHeading = NAN;


        // First face the expected weight accurately enough
        // that the centre/side weight sensors actually get a
        // fair chance to see it.
        if (fabsf(
                priorityHeadingError) >
            PRIORITY_TARGET_FINAL_ALIGN_DEG)
        {
            debugNav.print(
                "Priority target nearby: aligning "
            );

            debugNav.print(
                priorityHeadingError
            );

            debugNav.println(
                " deg"
            );


            motor_control_turn_relative(
                priorityHeadingError
            );


            roamingState =
                ROAM_TARGET_TURNING;

            return;
        }


        debugNav.print(
            "Priority target location reached: ("
        );

        debugNav.print(
            priorityX
        );

        debugNav.print(",");
        debugNav.print(
            priorityY
        );

        debugNav.println(
            ") - checking for weight"
        );


        priorityTargetWaitStartedAt =
            millis();

        roamingState =
            ROAM_TARGET_WAITING;

        return;
    }


    // ========================================================
    // EXISTING OBSTACLE TURN
    // ========================================================

    if (roamingState ==
        ROAM_TURNING)
    {
        if (motor_control_is_turning())
        {
            return;
        }


        // We have completed the avoidance turn.
        //
        // Do not point immediately back at the target.
        // Give the robot time to move around the obstacle.
        priorityRejoinAllowedAt =
            millis() +
            PRIORITY_TARGET_REJOIN_DELAY_MS;


        roamCheckStartedAt =
            millis();

        roamingState =
            ROAM_CHECKING;

        return;
    }


    // ========================================================
    // EXISTING NAVIGATION TOF READINGS
    // ========================================================

    int outerLeft =
        tof_get_nav_outer_left();

    int innerLeft =
        tof_get_nav_inner_left();

    int innerRight =
        tof_get_nav_inner_right();

    int outerRight =
        tof_get_nav_outer_right();


    outerLeft =
        clearanceValue(
            outerLeft
        );

    innerLeft =
        clearanceValue(
            innerLeft
        );

    innerRight =
        clearanceValue(
            innerRight
        );

    outerRight =
        clearanceValue(
            outerRight
        );


    int front =
        min(
            innerLeft,
            innerRight
        );


    int leftClearance =
        min(
            outerLeft,
            innerLeft
        );


    int rightClearance =
        min(
            outerRight,
            innerRight
        );


    // Keep your existing critical-distance protection.
    if (front <=
    ROAM_CRITICAL_MM)
    {
        motor_control_stop();


        debugNav.print(
            "Roaming: obstacle critically close, front="
        );

        debugNav.print(front);

        debugNav.println(
            " mm - triggering reverse escape"
        );


        reversing_set_reason(REVERSE_CRITICAL_OBSTACLE);
        setStateFlag(&STATE_FLAGS.reverse_triggered);
        return;
    }


    // ========================================================
    // ROAM STATE MACHINE
    // ========================================================

    switch (roamingState)
    {
        case ROAM_START:
        {
            motor_control_stop();

            roamCommandedPower = 0;
            roamCommandedHeading = NAN;


            roamCheckStartedAt =
                millis();

            roamingState =
                ROAM_CHECKING;

            break;
        }


        case ROAM_TURNING:
        {
            // Normally handled above.
            break;
        }


        case ROAM_TARGET_TURNING:
        {
            // Normally handled above.
            break;
        }


        case ROAM_TARGET_WAITING:
        {
            // Normally handled above.
            break;
        }


        case ROAM_CHECKING:
        {
            if (millis() -
                    roamCheckStartedAt <
                150)
            {
                return;
            }


            // Existing obstacle avoidance remains authoritative.
            if (front <
                    ROAM_FRONT_BLOCK_MM +
                        80 ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM +
                        30 ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM +
                        30)
            {
                roaming_start_turn(
                    leftClearance,
                    rightClearance
                );

                return;
            }


            // --------------------------------------------
            // Priority target steering
            // --------------------------------------------

            if (hasPriorityTarget &&
                millis() >=
                    priorityRejoinAllowedAt)
            {
                // If the target is significantly off-axis,
                // point-turn first instead of trying to make
                // a huge correction while driving at 430.
                if (fabsf(
                        priorityHeadingError) >
                    PRIORITY_TARGET_TURN_THRESHOLD_DEG)
                {
                    debugNav.print(
                        "Roaming: turn toward priority target "
                    );

                    debugNav.print(
                        priorityHeadingError
                    );

                    debugNav.print(
                        " deg, distance "
                    );

                    debugNav.println(
                        priorityDistance
                    );


                    motor_control_turn_relative(
                        priorityHeadingError
                    );


                    roamCommandedPower = 0;
                    roamCommandedHeading = NAN;


                    roamingState =
                        ROAM_TARGET_TURNING;

                    return;
                }


                // Convert the arena/pose heading error back
                // onto the absolute IMU heading expected by
                // motor_control_drive_heading().
                roamHeading =
                    imu_get_heading() +
                    priorityHeadingError;
            }
            else
            {
                // Existing roam behaviour, including the short
                // post-obstacle escape period.
                roamHeading =
                    imu_get_heading();
            }


            roamCommandedPower = 0;

            roamingState =
                ROAM_DRIVING;


            roaming_drive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            break;
        }


        case ROAM_DRIVING:
        {
            // --------------------------------------------
            // Existing wall avoidance has priority
            // --------------------------------------------

            if (front <
                    ROAM_FRONT_BLOCK_MM ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM)
            {
                roaming_start_turn(
                    leftClearance,
                    rightClearance
                );

                return;
            }


            // --------------------------------------------
            // Continually update bearing to priority target
            // once we've cleared any avoidance manoeuvre.
            // --------------------------------------------

            if (hasPriorityTarget &&
                millis() >=
                    priorityRejoinAllowedAt)
            {
                roamHeading =
                    imu_get_heading() +
                    priorityHeadingError;
            }


            roaming_drive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            break;
        }
    }
}


void frontier_targetting(){
    int frontier_x = get_frontier_x();
    int frontier_y = get_frontier_y();

    
}


void navigator_exe()
{
    if (!navigatorEnabled) return;

    NavState nav = getNavState();

    if (nav != lastNavState) {
        motor_control_stop();

        lastNavState = nav;
        navigatorStateStarted = millis();
        turnWatchActive = false;

        weight_detection_reset_side_evidence();

        if (nav == ROAMING)
        {
            roamingState = ROAM_START;
            pursuit_reset();
            roamCommandedPower = 0;
            roamCommandedHeading = NAN;
            priorityTargetWaitStartedAt = 0;
        }
        if (nav == REVERSING)
        {
            reversing_start();
        }
        if (nav == HOMING)
        {
            homing_start();

            debugNav.println("Navigator: HOMING started");
        }
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) {
        navigator_stop();
        // debugNav.println("Navigator stopped: IMU unavailable");
        return;
    }

    bool suppressPitchEscape = (nav == HOMING && homing_is_docking());

    if (reversing_check_for_pitch_escape(nav, suppressPitchEscape))
    {
        return;
    }

    if (!motor_control_is_turning()) turnWatchActive = false;

    if (turnWatchActive && millis() - turnStartedAt >= NAV_TURN_TIMEOUT_MS) {
        motor_control_stop();

        turnWatchActive = false;

        debugNav.println(
            "Navigation turn timeout - recovering"
        );

        if (nav == PURSUIT)
        {
            setStateFlag(
                &STATE_FLAGS.target_lost
            );
        }
        else if (nav == ROAMING)
        {
            roamingState =
                ROAM_START;
        }
        else if (nav == HOMING)
        {
            homing_start();
        }
        return;
    }


    switch (nav) {
        case ROAMING:
            roaming_exe();
            break;

        case PURSUIT:
            pursuit_update();
            break;

        case REVERSING:
            reversing_update();
            break;

        case HOMING:
            homing_update();
            break;

        default:
            break;
    }

    if (motor_control_is_turning() && !turnWatchActive) {
        turnStartedAt = millis();
        turnWatchActive = true;
    }
}

