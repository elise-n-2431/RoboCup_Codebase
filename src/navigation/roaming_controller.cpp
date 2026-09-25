#include "roaming_controller.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "map.h"
#include "pose.h"
#include "debug_print.h"
#include "priority_targets.h"

#include "navigation/weight_detection.h"
#include "navigation/pursuit_controller.h"
#include "navigation/reversing_controller.h"
#include "navigation/rejected_weights.h"


enum RoamingState
{
    ROAM_START,
    ROAM_CHECKING,
    ROAM_DRIVING,
    ROAM_TURNING,
    ROAM_TARGET_TURNING,
    ROAM_TARGET_WAITING,
    ROAM_FRONTIER
};

enum RoamGoalType
{
    ROAM_GOAL_FALLBACK,
    ROAM_GOAL_PRIORITY,
    ROAM_GOAL_FRONTIER
};

static RoamingState roamingState = ROAM_START;
static RoamGoalType roamGoal = ROAM_GOAL_FALLBACK;



static const int ROAM_POWER = 430;
static const int ROAM_SLOW_POWER = 340;

static const int ROAM_FRONT_BLOCK_MM = 150;
static const int ROAM_SIDE_BLOCK_MM = 180;
static const int ROAM_SLOW_MM = 250;
static const int ROAM_CRITICAL_MM = 90;

static const float ROAM_AVOID_TURN_DEG = 45.0f;


static const float PRIORITY_TARGET_ARRIVAL_MM = 250.0f;
static const float PRIORITY_TARGET_TURN_THRESHOLD_DEG = 18.0f;
static const float PRIORITY_TARGET_FINAL_ALIGN_DEG = 8.0f;

static const unsigned long PRIORITY_TARGET_CONFIRM_MS = 1200;
static const unsigned long PRIORITY_TARGET_REJOIN_DELAY_MS = 700;



static const unsigned long NAV_TELEMETRY_PERIOD_MS = 200;
static unsigned long lastNavTelemetryAt = 0;



static bool roamingPickupEnabled = false;

static unsigned long roamingStartedAt = 0;
static unsigned long lastWeightCheckAt = 0;
static unsigned long roamCheckStartedAt = 0;

static unsigned long priorityTargetWaitStartedAt = 0;
static unsigned long priorityRejoinAllowedAt = 0;

static float roamHeading = 0.0f;
static float roamCommandedHeading = NAN;
static int roamCommandedPower = 0;

static int roamTurnDirection = 1;

static float roamGoalX = -1.0f;
static float roamGoalY = -1.0f;

static bool frontierReachedLogged = false;


static float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    return angle;
}


static int clearanceValue(int distance)
{
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}


static const char* roamingStateName()
{
    switch (roamingState)
    {
        case ROAM_START: return "START";
        case ROAM_CHECKING: return "CHECKING";
        case ROAM_DRIVING: return "DRIVING";
        case ROAM_TURNING: return "TURNING";
        case ROAM_TARGET_TURNING: return "TARGET_TURNING";
        case ROAM_TARGET_WAITING: return "TARGET_WAITING";
        case ROAM_FRONTIER: return "FRONTIER";
        default: return "UNKNOWN";
    }
}


static const char* roamGoalName()
{
    switch (roamGoal)
    {
        case ROAM_GOAL_PRIORITY: return "PRIORITY";
        case ROAM_GOAL_FRONTIER: return "FRONTIER";
        default: return "FALLBACK";
    }
}


static const char* weightTargetName(WeightTargetSide target)
{
    switch (target)
    {
        case TARGET_LEFT: return "LEFT";
        case TARGET_RIGHT: return "RIGHT";
        case TARGET_CENTRE: return "CENTRE";
        default: return "NONE";
    }
}


static bool setRoamGoal(
    RoamGoalType newGoal,
    float x,
    float y)
{
    bool changed =
        newGoal != roamGoal ||
        fabsf(x - roamGoalX) > 1.0f ||
        fabsf(y - roamGoalY) > 1.0f;

    if (!changed)
    {
        return false;
    }

    roamGoal = newGoal;
    roamGoalX = x;
    roamGoalY = y;

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",GOAL,");
    debugNav.print(roamGoalName());
    debugNav.print(",");
    debugNav.print(roamGoalX);
    debugNav.print(",");
    debugNav.println(roamGoalY);

    return true;
}


static bool getPriorityTargetInfo(
    float &targetX,
    float &targetY,
    float &distance,
    float &headingError)
{
    if (!roamingPickupEnabled)
    {
        return false;
    }

    if (priority_targets_count() == 0)
    {
        return false;
    }

    if (!priority_targets_get(0, targetX, targetY))
    {
        return false;
    }

    float dx = targetX - pose_get_x_mm();
    float dy = targetY - pose_get_y_mm();

    distance = sqrtf(dx * dx + dy * dy);

    float desiredPoseHeading =
        atan2f(dy, dx) *
        180.0f / PI;

    headingError =
        wrap180(
            desiredPoseHeading -
            pose_get_heading_deg()
        );

    return true;
}


static void readClearances(
    int &outerLeft,
    int &innerLeft,
    int &innerRight,
    int &outerRight,
    int &front,
    int &leftClearance,
    int &rightClearance)
{
    outerLeft =
        clearanceValue(
            tof_get_nav_outer_left()
        );

    innerLeft =
        clearanceValue(
            tof_get_nav_inner_left()
        );

    innerRight =
        clearanceValue(
            tof_get_nav_inner_right()
        );

    outerRight =
        clearanceValue(
            tof_get_nav_outer_right()
        );

    front = min(
        innerLeft,
        innerRight
    );

    leftClearance = min(
        outerLeft,
        innerLeft
    );

    rightClearance = min(
        outerRight,
        innerRight
    );
}


static void printRoamingTelemetry(
    int front,
    int leftClearance,
    int rightClearance,
    bool hasPriorityTarget,
    bool hasFrontierTarget)
{
    if (!debugNav.enabled)
    {
        return;
    }

    if (millis() - lastNavTelemetryAt <
        NAV_TELEMETRY_PERIOD_MS)
    {
        return;
    }

    lastNavTelemetryAt = millis();

    debugNav.print("NAV,");
    debugNav.print(millis());

    debugNav.print(",ROAMING,");

    debugNav.print(roamingStateName());

    debugNav.print(",");
    debugNav.print(roamGoalName());

    debugNav.print(",");
    debugNav.print(roamGoalX);

    debugNav.print(",");
    debugNav.print(roamGoalY);

    debugNav.print(",");
    debugNav.print(front);

    debugNav.print(",");
    debugNav.print(leftClearance);

    debugNav.print(",");
    debugNav.print(rightClearance);

    debugNav.print(",");
    debugNav.print(
        hasPriorityTarget ? 1 : 0
    );

    debugNav.print(",");
    debugNav.print(
        hasFrontierTarget ? 1 : 0
    );

    debugNav.print(",");
    debugNav.println(
        (int)priority_targets_count()
    );
}


static bool checkForWeight()
{
    if (!roamingPickupEnabled)
    {
        return false;
    }

    if (millis() - roamingStartedAt < 1000)
    {
        return false;
    }

    if (millis() - lastWeightCheckAt < 100)
    {
        return false;
    }

    lastWeightCheckAt = millis();

    WeightTargetSide detectedTarget =
        weight_detection_update();

    if (detectedTarget == TARGET_NONE)
    {
        return false;
    }
    float rejectedDistance = -1.0f;

    if (rejected_weights_is_near(
            pose_get_x_mm(),
            pose_get_y_mm(),
            rejectedDistance))
        {
        debugNav.print("NAV_EVENT,");
        debugNav.print(millis());
        debugNav.print(",DUMMY_IGNORED,");
        debugNav.print(rejectedDistance);
        debugNav.print(",");
        debugNav.println(
            weightTargetName(
                detectedTarget
            )
        );

        weight_detection_block_for(
            1000
        );

        return false;
    }



    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",WEIGHT_DETECTED,");
    debugNav.println(
        weightTargetName(detectedTarget)
    );

    pursuit_start(detectedTarget);

    motor_control_stop();

    setStateFlag(
        &STATE_FLAGS.target_identified
    );

    return true;
}



static void roamingStartTurn(
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

    roamingState = ROAM_TURNING;

    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    priorityTargetWaitStartedAt = 0;

    weight_detection_reset_side_evidence();

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",ROAM_TURN,");
    debugNav.println(
        roamTurnDirection < 0
            ? "LEFT"
            : "RIGHT"
    );
}


static void roamingDrive(int power)
{
    bool sameHeading =
        isfinite(roamCommandedHeading) &&
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

    roamCommandedPower = power;
    roamCommandedHeading = roamHeading;
}


static bool handleActiveTurn()
{
    if (roamingState == ROAM_TARGET_TURNING)
    {
        if (motor_control_is_turning())
        {
            return true;
        }

        motor_control_stop();

        roamCommandedPower = 0;
        roamCommandedHeading = NAN;

        roamCheckStartedAt = millis();
        roamingState = ROAM_CHECKING;

        return true;
    }

    if (roamingState == ROAM_TURNING)
    {
        if (motor_control_is_turning())
        {
            return true;
        }

        priorityRejoinAllowedAt =
            millis() +
            PRIORITY_TARGET_REJOIN_DELAY_MS;

        roamCheckStartedAt = millis();
        roamingState = ROAM_CHECKING;

        return true;
    }

    return false;
}


static bool handlePriorityWaiting(
    bool hasPriorityTarget,
    float priorityX,
    float priorityY)
{
    if (roamingState != ROAM_TARGET_WAITING)
    {
        return false;
    }

    if (!hasPriorityTarget)
    {
        priorityTargetWaitStartedAt = 0;
        roamingState = ROAM_START;

        return true;
    }

    if (priorityTargetWaitStartedAt == 0)
    {
        priorityTargetWaitStartedAt =
            millis();
    }

    if (millis() -
            priorityTargetWaitStartedAt <
        PRIORITY_TARGET_CONFIRM_MS)
    {
        return true;
    }

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",PRIORITY_EMPTY,");
    debugNav.print(priorityX);
    debugNav.print(",");
    debugNav.println(priorityY);

    priority_targets_remove(0);

    priorityTargetWaitStartedAt = 0;
    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    roamingState = ROAM_START;

    return true;
}


static void updatePriorityTarget(
    float priorityX,
    float priorityY,
    float priorityDistance,
    float priorityHeadingError,
    int outerLeft,
    int outerRight,
    int front,
    int leftClearance,
    int rightClearance)
{
    if (priorityDistance <=
        PRIORITY_TARGET_ARRIVAL_MM)
    {
        motor_control_stop();

        roamCommandedPower = 0;
        roamCommandedHeading = NAN;

        if (fabsf(priorityHeadingError) >
            PRIORITY_TARGET_FINAL_ALIGN_DEG)
        {
            debugNav.print("NAV_EVENT,");
            debugNav.print(millis());
            debugNav.print(",PRIORITY_ALIGN,");
            debugNav.println(
                priorityHeadingError
            );

            motor_control_turn_relative(
                -priorityHeadingError
            );

            roamingState =
                ROAM_TARGET_TURNING;

            return;
        }

        debugNav.print("NAV_EVENT,");
        debugNav.print(millis());
        debugNav.print(",PRIORITY_REACHED,");
        debugNav.print(priorityX);
        debugNav.print(",");
        debugNav.println(priorityY);

        priorityTargetWaitStartedAt =
            millis();

        roamingState =
            ROAM_TARGET_WAITING;

        return;
    }

    switch (roamingState)
    {
        case ROAM_START:
        {
            motor_control_stop();

            roamCommandedPower = 0;
            roamCommandedHeading = NAN;

            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;

            return;
        }

        case ROAM_CHECKING:
        {
            if (millis() -
                    roamCheckStartedAt <
                150)
            {
                return;
            }

            if (front <
                    ROAM_FRONT_BLOCK_MM + 80 ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM + 30 ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM + 30)
            {
                roamingStartTurn(
                    leftClearance,
                    rightClearance
                );

                return;
            }

            if (millis() >=
                    priorityRejoinAllowedAt &&
                fabsf(priorityHeadingError) >
                    PRIORITY_TARGET_TURN_THRESHOLD_DEG)
            {
                debugNav.print("NAV_EVENT,");
                debugNav.print(millis());
                debugNav.print(",PRIORITY_TURN,");
                debugNav.print(priorityHeadingError);
                debugNav.print(",");
                debugNav.println(priorityDistance);

                motor_control_turn_relative(
                    -priorityHeadingError
                );

                roamCommandedPower = 0;
                roamCommandedHeading = NAN;

                roamingState =
                    ROAM_TARGET_TURNING;

                return;
            }

            if (millis() >=
                priorityRejoinAllowedAt)
            {
                roamHeading =
                    imu_get_heading() +
                    priorityHeadingError;
            }
            else
            {
                roamHeading =
                    imu_get_heading();
            }

            roamCommandedPower = 0;
            roamingState = ROAM_DRIVING;

            roamingDrive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            return;
        }

        case ROAM_DRIVING:
        {
            if (front <
                    ROAM_FRONT_BLOCK_MM ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM)
            {
                roamingStartTurn(
                    leftClearance,
                    rightClearance
                );

                return;
            }

            if (millis() >=
                priorityRejoinAllowedAt)
            {
                roamHeading =
                    imu_get_heading() +
                    priorityHeadingError;
            }

            roamingDrive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            return;
        }

        default:
        {
            roamingState = ROAM_START;
            return;
        }
    }
}



static void updateFrontierTarget(
    float frontierX,
    float frontierY)
{
    bool changed =
        setRoamGoal(
            ROAM_GOAL_FRONTIER,
            frontierX,
            frontierY
        );

    if (changed)
    {
        frontierReachedLogged = false;
    }

    roamingState = ROAM_FRONTIER;

    motor_control_drive_to_point(
        frontierX,
        frontierY,
        ROAM_POWER
    );

    if (motor_control_point_reached() &&
        !frontierReachedLogged)
    {
        frontierReachedLogged = true;

        debugNav.print("NAV_EVENT,");
        debugNav.print(millis());
        debugNav.print(",FRONTIER_REACHED,");
        debugNav.print(frontierX);
        debugNav.print(",");
        debugNav.println(frontierY);
    }
}



static void updateFallbackRoaming(
    int outerLeft,
    int outerRight,
    int front,
    int leftClearance,
    int rightClearance)
{
    switch (roamingState)
    {
        case ROAM_START:
        {
            motor_control_stop();

            roamCommandedPower = 0;
            roamCommandedHeading = NAN;

            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;

            return;
        }

        case ROAM_CHECKING:
        {
            if (millis() -
                    roamCheckStartedAt <
                150)
            {
                return;
            }

            if (front <
                    ROAM_FRONT_BLOCK_MM + 80 ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM + 30 ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM + 30)
            {
                roamingStartTurn(
                    leftClearance,
                    rightClearance
                );

                return;
            }

            roamHeading =
                imu_get_heading();

            roamCommandedPower = 0;
            roamingState = ROAM_DRIVING;

            roamingDrive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            return;
        }

        case ROAM_DRIVING:
        {
            if (front <
                    ROAM_FRONT_BLOCK_MM ||
                outerLeft <
                    ROAM_SIDE_BLOCK_MM ||
                outerRight <
                    ROAM_SIDE_BLOCK_MM)
            {
                roamingStartTurn(
                    leftClearance,
                    rightClearance
                );

                return;
            }

            roamingDrive(
                front < ROAM_SLOW_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER
            );

            return;
        }

        default:
        {
            roamingState = ROAM_START;
            return;
        }
    }
}


static bool checkCriticalObstacle(int front)
{
    if (front > ROAM_CRITICAL_MM)
    {
        return false;
    }

    motor_control_stop();

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",CRITICAL_REVERSE,");
    debugNav.println(front);

    reversing_set_reason(
        REVERSE_CRITICAL_OBSTACLE
    );

    setStateFlag(
        &STATE_FLAGS.reverse_triggered
    );

    return true;
}



void roaming_reset()
{
    roamingState = ROAM_START;
    roamGoal = ROAM_GOAL_FALLBACK;

    roamingStartedAt = millis();
    lastWeightCheckAt = millis();
    roamCheckStartedAt = 0;

    priorityTargetWaitStartedAt = 0;
    priorityRejoinAllowedAt = 0;

    roamHeading = 0.0f;
    roamCommandedHeading = NAN;
    roamCommandedPower = 0;

    roamGoalX = -1.0f;
    roamGoalY = -1.0f;

    frontierReachedLogged = false;

    weight_detection_reset_side_evidence();
}


void roaming_start(bool pickupEnabled)
{
    roamingPickupEnabled = pickupEnabled;

    roaming_reset();

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",ROAM_START,");
    debugNav.print(
        roamingPickupEnabled ? 1 : 0
    );
    debugNav.print(",");
    debugNav.println(
        (int)priority_targets_count()
    );
}


void roaming_turn_timeout()
{
    motor_control_stop();

    roamingState = ROAM_START;

    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    priorityTargetWaitStartedAt = 0;
    priorityRejoinAllowedAt = 0;

    weight_detection_reset_side_evidence();

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.println(",ROAM_TURN_TIMEOUT");
}


void roaming_update()
{
    // Physical weight detection always has first priority.
    if (checkForWeight())
    {
        return;
    }

    // --------------------------------------------------------
    // Current priority target
    // --------------------------------------------------------

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

    // --------------------------------------------------------
    // Current map frontier
    // --------------------------------------------------------

    float frontierX = -1.0f;
    float frontierY = -1.0f;

    bool hasFrontierTarget =
        get_frontier_target(
            frontierX,
            frontierY
        );

    // --------------------------------------------------------
    // Current clearances
    // --------------------------------------------------------

    int outerLeft;
    int innerLeft;
    int innerRight;
    int outerRight;
    int front;
    int leftClearance;
    int rightClearance;

    readClearances(
        outerLeft,
        innerLeft,
        innerRight,
        outerRight,
        front,
        leftClearance,
        rightClearance
    );

    printRoamingTelemetry(
        front,
        leftClearance,
        rightClearance,
        hasPriorityTarget,
        hasFrontierTarget
    );

    // Finish any point-turn already in progress before changing
    // roaming goal.
    if (handleActiveTurn())
    {
        return;
    }

    // Priority target waiting is also allowed to continue while
    // physical weight detection runs at the top of this function.
    if (handlePriorityWaiting(
            hasPriorityTarget,
            priorityX,
            priorityY))
    {
        return;
    }

    // Extremely close obstacle always gets the dedicated reverse.
    if (checkCriticalObstacle(front))
    {
        return;
    }


    if (hasPriorityTarget)
    {
        if (roamGoal != ROAM_GOAL_PRIORITY)
        {
            motor_control_stop();

            roamingState = ROAM_START;
            roamCommandedPower = 0;
            roamCommandedHeading = NAN;
        }

        setRoamGoal(
            ROAM_GOAL_PRIORITY,
            priorityX,
            priorityY
        );

        updatePriorityTarget(
            priorityX,
            priorityY,
            priorityDistance,
            priorityHeadingError,
            outerLeft,
            outerRight,
            front,
            leftClearance,
            rightClearance
        );

        return;
    }

 
    if (hasFrontierTarget)
    {
        if (roamGoal != ROAM_GOAL_FRONTIER)
        {
            motor_control_stop();

            roamingState = ROAM_FRONTIER;
            roamCommandedPower = 0;
            roamCommandedHeading = NAN;
        }

        updateFrontierTarget(
            frontierX,
            frontierY
        );

        return;
    }

    if (roamGoal != ROAM_GOAL_FALLBACK)
    {
        motor_control_stop();

        roamingState = ROAM_START;
        roamCommandedPower = 0;
        roamCommandedHeading = NAN;

        setRoamGoal(
            ROAM_GOAL_FALLBACK,
            -1.0f,
            -1.0f
        );
    }

    updateFallbackRoaming(
        outerLeft,
        outerRight,
        front,
        leftClearance,
        rightClearance
    );
}