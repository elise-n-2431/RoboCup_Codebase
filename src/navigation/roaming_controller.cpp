#include "roaming_controller.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "map.h"
#include "arena_config.h"
#include "pose.h"
#include "debug_print.h"
#include "priority_targets.h"
#include "path_finding.h"

#include "navigation/weight_detection.h"
#include "navigation/pursuit_controller.h"
#include "navigation/reversing_controller.h"
#include "navigation/rejected_weights.h"


enum RoamingState
{
    ROAM_START,
    ROAM_DSTAR,
    // D* requested a large heading change.
    // Turn only a small amount, then replan.
    ROAM_DSTAR_STEP_TURNING,

    ROAM_TARGET_TURNING,
    ROAM_TARGET_WAITING
};

enum RoamGoalType
{
    ROAM_GOAL_NONE,
    ROAM_GOAL_PRIORITY,
    ROAM_GOAL_FRONTIER,
    ROAM_GOAL_OPEN_SPACE
};

static RoamingState roamingState =
    ROAM_START;

static RoamGoalType roamGoal =
    ROAM_GOAL_NONE;

static const int ROAM_POWER = 450;
static const int ROAM_SLOW_POWER = 400;

static const float ROAM_SLOW_DISTANCE_MM = 700.0f;
static const int ROAM_SLOW_MM = 250;
static const int CRITICAL_OBSTACLE_MM = 90;

static const float FRONTIER_TARGET_ARRIVAL_MM = 120.0f;

static const float ROAM_DSTAR_SLOW_DISTANCE_MM = 300.0f;

// If a D* waypoint requires a very large change of heading,
// do not immediately perform the whole point turn.
static const float DSTAR_LARGE_TURN_DEG = 45.0f;
static const int OPEN_SPACE_SEARCH_CELLS = 20;
static const float OPEN_SPACE_MIN_DISTANCE_MM = 350.0f;
static const float OPEN_SPACE_MAX_DISTANCE_MM = 1000.0f;
static const float OPEN_SPACE_PREVIOUS_EXCLUSION_MM = 300.0f;
static const unsigned long OPEN_SPACE_RETRY_MS = 1000;

// Rotate only this much, allow the map to update,
// then ask D* for a fresh route.
static const float DSTAR_TURN_STEP_DEG = 30.0f;

static const float PRIORITY_TARGET_ARRIVAL_MM = 80.0f;
static const float PRIORITY_TARGET_FINAL_ALIGN_DEG = 8.0f;

static const unsigned long PRIORITY_TARGET_CONFIRM_MS = 1200;

// If D* cannot currently reach a priority target,
// don't hammer the same failed target every loop.
// Explore a frontier for a while and try again later.
static const unsigned long PRIORITY_RETRY_DELAY_MS = 5000;

static unsigned long priorityFailedAt[MAX_PRIORITY_TARGETS] = {};
static int priorityTargetIndex = -1;
static float priorityRawX = -1.0f;
static float priorityRawY = -1.0f;

static const unsigned long NAV_TELEMETRY_PERIOD_MS = 200;

static unsigned long lastNavTelemetryAt = 0;

static const float PRIORITY_SEARCH_ANGLE_DEG = 12.0f;
static const unsigned long PRIORITY_SEARCH_SETTLE_MS = 200;
static const unsigned long FANNING_PERIOD_MS = 10000;
static unsigned long PREV_FAN_TIME_MS = 0;
static const float FAN_ANGLE_DEG = 90;
static const unsigned long FAN_DURATION_MS = 1500;


static int prioritySearchStage = 0;
static unsigned long prioritySearchStageAt = 0;



static bool roamingPickupEnabled = false;

static unsigned long roamingStartedAt = 0;
static unsigned long lastWeightCheckAt = 0;

static unsigned long priorityTargetWaitStartedAt = 0;

static float roamCommandedHeading = NAN;
static int roamCommandedPower = 0;

static float roamGoalX = -1.0f;
static float roamGoalY = -1.0f;

static bool frontierReachedLogged = false;
static float lastOpenGoalX = -10000.0f;
static float lastOpenGoalY = -10000.0f;
static unsigned long lastOpenSearchAt = 0;


static float wrap180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    return angle;
}

FLASHMEM static bool confirmedOpenCell(int x, int y)
{
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
            if (!check_free(x + dx, y + dy)) return false;
    return !cell_too_close_to_obstacle(x, y);
}

FLASHMEM static bool confirmedOpenCorridor(float goalX, float goalY)
{
    float startX = pose_get_x_mm();
    float startY = pose_get_y_mm();
    float dx = goalX - startX;
    float dy = goalY - startY;
    float distance = sqrtf(dx * dx + dy * dy);
    // Leave the robot's occupied cell out of the free-map requirement.
    for (float travelled = 150.0f; travelled <= distance;
         travelled += 50.0f)
    {
        float fraction = travelled / distance;
        if (!confirmedOpenCell(
                world_to_cell_x(startX + dx * fraction),
                world_to_cell_y(startY + dy * fraction))) return false;
    }
    return confirmedOpenCell(world_to_cell_x(goalX),
                             world_to_cell_y(goalY));
}

FLASHMEM static bool chooseNearbyOpenSpace(float &goalX, float &goalY)
{
    int currentX = world_to_cell_x(pose_get_x_mm());
    int currentY = world_to_cell_y(pose_get_y_mm());
    float bestScore = -1e9f;
    bool found = false;
    for (int x = currentX - OPEN_SPACE_SEARCH_CELLS;
         x <= currentX + OPEN_SPACE_SEARCH_CELLS; x += 2)
    {
        for (int y = currentY - OPEN_SPACE_SEARCH_CELLS;
             y <= currentY + OPEN_SPACE_SEARCH_CELLS; y += 2)
        {
            float xMm = cell_to_world_x(x);
            float yMm = cell_to_world_y(y);
            if (xMm < 150.0f || xMm > ARENA_X_MM - 150.0f ||
                yMm < 150.0f || yMm > ARENA_Y_MM - 150.0f) continue;
            float dx = xMm - pose_get_x_mm();
            float dy = yMm - pose_get_y_mm();
            float distance = sqrtf(dx * dx + dy * dy);
            if (distance < OPEN_SPACE_MIN_DISTANCE_MM ||
                distance > OPEN_SPACE_MAX_DISTANCE_MM ||
                hypotf(xMm - lastOpenGoalX, yMm - lastOpenGoalY) <
                    OPEN_SPACE_PREVIOUS_EXCLUSION_MM ||
                !confirmedOpenCell(x, y)) continue;

            float headingError = wrap180(atan2f(dy, dx) * 180.0f / PI -
                                         pose_get_heading_deg());
            if (fabsf(headingError) > 60.0f ||
                !confirmedOpenCorridor(xMm, yMm)) continue;

            int openCells = 0;
            for (int ox = -3; ox <= 3; ox++)
                for (int oy = -3; oy <= 3; oy++)
                    if (check_free(x + ox, y + oy)) openCells++;
            float score = openCells * 15.0f -
                          fabsf(distance - 750.0f) * 0.2f -
                          fabsf(headingError) * 3.0f;
            if (score > bestScore)
            {
                bestScore = score;
                goalX = xMm;
                goalY = yMm;
                found = true;
            }
        }
    }
    return found;
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
        case ROAM_START:
            return "START";

        case ROAM_DSTAR:
            return "DSTAR";

        case ROAM_TARGET_TURNING:
            return "TARGET_TURNING";

        case ROAM_TARGET_WAITING:
            return "TARGET_WAITING";

        case ROAM_DSTAR_STEP_TURNING:
            return "DSTAR_STEP_TURN";

        default:
            return "UNKNOWN";
    }
}

static const char* roamGoalName()
{
    switch (roamGoal)
    {
        case ROAM_GOAL_PRIORITY:
            return "PRIORITY";

        case ROAM_GOAL_FRONTIER:
            return "FRONTIER";
        case ROAM_GOAL_OPEN_SPACE:
            return "OPEN_SPACE";

        default:
            return "NONE";
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


FLASHMEM static bool setRoamGoal(
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

FLASHMEM static bool startRoamDstar(
    RoamGoalType goalType,
    float goalX,
    float goalY,
    bool keepDriving = false)
{
    if (!keepDriving) motor_control_stop();

    path_reset();

    roamCommandedPower = 0;
    roamCommandedHeading = NAN;


    setRoamGoal(
        goalType,
        goalX,
        goalY
    );


    if (!path_init(goalX, goalY))
    {
        motor_control_stop();
        if (goalType == ROAM_GOAL_OPEN_SPACE)
        {
            lastOpenGoalX = goalX;
            lastOpenGoalY = goalY;
            debugNav.print("OPEN_SPACE_UNREACHABLE,");
            debugNav.print(goalX);
            debugNav.print(",");
            debugNav.println(goalY);
        }
        // Don't immediately retry an unreachable priority
        // hundreds of times per second.
        if (goalType == ROAM_GOAL_PRIORITY)
        {
            if (priorityTargetIndex >= 0)
                priorityFailedAt[priorityTargetIndex] = millis();

            debugNav.print("NAV_EVENT,");
            debugNav.print(millis());
            debugNav.println(",PRIORITY_RETRY_BLOCKED");
        }

        debugNav.print(
            "NAV_EVENT,"
        );

        debugNav.print(
            "NAV_EVENT,"
        );

        debugNav.print(
            millis()
        );

        debugNav.print(
            ",ROAM_DSTAR_INIT_FAILED,"
        );

        debugNav.println(
            roamGoalName()
        );


        path_reset();

        setRoamGoal(
            ROAM_GOAL_NONE,
            -1.0f,
            -1.0f
        );

        roamingState =
            ROAM_START;

        return false;
    }


    roamingState =
        ROAM_DSTAR;
        //no longer blocked


    debugNav.print(
        "NAV_EVENT,"
    );

    debugNav.print(
        millis()
    );

    debugNav.print(
        ",ROAM_DSTAR_START,"
    );

    debugNav.print(
        roamGoalName()
    );

    debugNav.print(",");

    debugNav.print(
        roamGoalX
    );

    debugNav.print(",");

    debugNav.println(
        roamGoalY
    );


    return true;
}
static float roamGoalDistance()
{
    float dx =
        roamGoalX -
        pose_get_x_mm();

    float dy =
        roamGoalY -
        pose_get_y_mm();

    return sqrtf(
        dx * dx +
        dy * dy
    );
}


static float roamGoalHeadingError()
{
    float dx =
        (roamGoal == ROAM_GOAL_PRIORITY ? priorityRawX : roamGoalX) -
        pose_get_x_mm();

    float dy =
        (roamGoal == ROAM_GOAL_PRIORITY ? priorityRawY : roamGoalY) -
        pose_get_y_mm();

    float desiredHeading =
        atan2f(dy, dx) *
        180.0f / PI;

    return wrap180(
        desiredHeading -
        pose_get_heading_deg()
    );
}

FLASHMEM static bool getPriorityTargetInfo(
    int &targetIndex,
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

    targetIndex = -1;
    distance = INFINITY;
    for (uint8_t i = 0; i < priority_targets_count(); i++)
    {
        float x, y;
        if (!priority_targets_get(i, x, y)) continue;
        float candidateDistance = hypotf(x - pose_get_x_mm(),
                                         y - pose_get_y_mm());
        // Keep the current target stable while its route or scan is active.
        if (roamGoal == ROAM_GOAL_PRIORITY &&
            i == priorityTargetIndex &&
            fabsf(x - priorityRawX) < 1.0f &&
            fabsf(y - priorityRawY) < 1.0f)
        {
            targetIndex = i;
            targetX = x;
            targetY = y;
            break;
        }
        if (priorityFailedAt[i] != 0 &&
            millis() - priorityFailedAt[i] < PRIORITY_RETRY_DELAY_MS)
            continue;
        if (candidateDistance < distance)
        {
            distance = candidateDistance;
            targetIndex = i;
            targetX = x;
            targetY = y;
        }
    }
    if (targetIndex < 0) return false;

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


bool roaming_check_for_weight()
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
    unsigned long rejectedAge = 0;

    if (rejected_weights_is_near(
            pose_get_x_mm(),
            pose_get_y_mm(),
            rejectedDistance,
            rejectedAge))
        {
        debugNav.print("WEIGHT_REJECTED_KNOWN_DUMMY,");
        debugNav.print(rejectedDistance);
        debugNav.print(",");
        debugNav.println(rejectedAge);
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

FLASHMEM static bool handlePriorityWaiting()
{
    if (roamingState !=
        ROAM_TARGET_WAITING)
    {
        return false;
    }

    float priorityX, priorityY;
    if (priorityTargetIndex < 0 ||
        !priority_targets_get(priorityTargetIndex, priorityX, priorityY) ||
        fabsf(priorityX - priorityRawX) > 1.0f ||
        fabsf(priorityY - priorityRawY) > 1.0f)
    {
        priorityTargetWaitStartedAt = 0;
        prioritySearchStage = 0;
        roamingState = ROAM_START;

        return true;
    }

    // Stage 0:
    // Give the sensors a moment while stationary.
    if (prioritySearchStage == 0)
    {
        if (millis() -
                prioritySearchStageAt <
            PRIORITY_SEARCH_SETTLE_MS)
        {
            return true;
        }

        motor_control_turn_relative(
            -PRIORITY_SEARCH_ANGLE_DEG
        );

        prioritySearchStage = 1;

        debugNav.println(
            "NAV_EVENT,PRIORITY_SEARCH,LEFT"
        );

        return true;
    }

    // Wait for first small turn.
    if (prioritySearchStage == 1)
    {
        if (motor_control_is_turning())
        {
            return true;
        }

        motor_control_stop();

        prioritySearchStage = 2;
        prioritySearchStageAt = millis();

        return true;
    }

    // Small pause at left extreme.
    if (prioritySearchStage == 2)
    {
        if (millis() -
                prioritySearchStageAt <
            PRIORITY_SEARCH_SETTLE_MS)
        {
            return true;
        }

        motor_control_turn_relative(
            2.0f *
            PRIORITY_SEARCH_ANGLE_DEG
        );

        prioritySearchStage = 3;

        debugNav.println(
            "NAV_EVENT,PRIORITY_SEARCH,RIGHT"
        );

        return true;
    }

    // Wait for right turn.
    if (prioritySearchStage == 3)
    {
        if (motor_control_is_turning())
        {
            return true;
        }

        motor_control_stop();

        prioritySearchStage = 4;
        prioritySearchStageAt = millis();

        return true;
    }

    // Pause at right extreme.
    if (prioritySearchStage == 4)
    {
        if (millis() -
                prioritySearchStageAt <
            PRIORITY_SEARCH_SETTLE_MS)
        {
            return true;
        }

        // Return to original heading.
        motor_control_turn_relative(
            -PRIORITY_SEARCH_ANGLE_DEG
        );

        prioritySearchStage = 5;

        return true;
    }

    if (prioritySearchStage == 5)
    {
        if (motor_control_is_turning())
        {
            return true;
        }

        motor_control_stop();

        prioritySearchStage = 6;
        prioritySearchStageAt = millis();

        return true;
    }

    // Final stationary look.
    if (millis() -
            prioritySearchStageAt <
        PRIORITY_SEARCH_SETTLE_MS)
    {
        return true;
    }

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",PRIORITY_EMPTY,");
    debugNav.print(priorityX);
    debugNav.print(",");
    debugNav.println(priorityY);

    priority_targets_remove(priorityTargetIndex);
    for (int i = priorityTargetIndex; i + 1 < MAX_PRIORITY_TARGETS; i++)
        priorityFailedAt[i] = priorityFailedAt[i + 1];
    priorityFailedAt[MAX_PRIORITY_TARGETS - 1] = 0;
    priorityTargetIndex = -1;

    priorityTargetWaitStartedAt = 0;
    prioritySearchStage = 0;

    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    roamingState = ROAM_START;

    return true;
}


FLASHMEM static void updatePriorityTarget(
    int targetIndex,
    float priorityX,
    float priorityY,
    float priorityDistance)
{
    priorityTargetIndex = targetIndex;
    priorityRawX = priorityX;
    priorityRawY = priorityY;
    debugNav.print("PRIORITY_CHOICE,");
    debugNav.print(targetIndex);
    debugNav.print(",");
    debugNav.print(priorityX);
    debugNav.print(",");
    debugNav.print(priorityY);
    debugNav.print(",");
    debugNav.println(priorityDistance);

    // Map cells are 50 mm, with padding outside the arena and a two-cell
    // obstacle clearance. An edge weight can be valid while its robot-centre
    // goal is not. Try a few inboard cells and retain the raw target above.
    for (float margin = 250.0f; margin <= 400.0f; margin += 50.0f)
    {
        float approachX = priorityX;
        float approachY = priorityY;
        if (priorityX < margin) approachX = margin;
        else if (priorityX > ARENA_X_MM - margin)
            approachX = ARENA_X_MM - margin;
        if (priorityY < margin) approachY = margin;
        else if (priorityY > ARENA_Y_MM - margin)
            approachY = ARENA_Y_MM - margin;
        int cellX = world_to_cell_x(approachX);
        int cellY = world_to_cell_y(approachY);
        if (check_obstacle(cellX, cellY) ||
            cell_too_close_to_obstacle(cellX, cellY)) continue;
        if (startRoamDstar(ROAM_GOAL_PRIORITY, approachX, approachY))
            return;
    }
    priorityFailedAt[targetIndex] = millis();
    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.println(",PRIORITY_RETRY_BLOCKED");
    setRoamGoal(ROAM_GOAL_NONE, -1.0f, -1.0f);
    roamingState = ROAM_START;
}

static void updateFrontierTarget(
    float frontierX,
    float frontierY,
    bool keepDriving = false)
{
    frontierReachedLogged =
        false;

    startRoamDstar(
        ROAM_GOAL_FRONTIER,
        frontierX,
        frontierY,
        keepDriving
    );
}


static bool checkCriticalObstacle(int front)
{
    if (front <= 0)
    {
        return false;
    }

    if (front > CRITICAL_OBSTACLE_MM)
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

void perform_fan_operation() {
    motor_control_turn_relative(
    FAN_ANGLE_DEG
    );
    
    int start_time = millis();
    while (millis() < (start_time + FAN_DURATION_MS)) {
    }

    motor_control_turn_relative(
    -FAN_ANGLE_DEG
    );
        
    start_time = millis();
    while (millis() < (start_time + FAN_DURATION_MS)) {
    }
}


void roaming_reset()
{
    path_reset();

    roamingState =
        ROAM_START;

    roamGoal =
        ROAM_GOAL_NONE;


    roamingStartedAt =
        millis();

    lastWeightCheckAt =
        millis();

    priorityTargetWaitStartedAt = 0;

    prioritySearchStage = 0;
    prioritySearchStageAt = 0;


    roamCommandedHeading = NAN;
    roamCommandedPower = 0;

    roamGoalX = -1.0f;
    roamGoalY = -1.0f;


    frontierReachedLogged =
        false;


    weight_detection_reset_side_evidence();
}


void roaming_start(bool pickupEnabled)
{
    roamingPickupEnabled = pickupEnabled;
    PREV_FAN_TIME_MS = millis();

    roaming_reset();
    for (int i = 0; i < MAX_PRIORITY_TARGETS; i++)
        priorityFailedAt[i] = 0;
    priorityTargetIndex = -1;

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


void roaming_update()
{

    if (getNavState() != ROAMING) {
        return;
    }
    // ========================================================
    // PHYSICAL WEIGHT DETECTION
    //
    // A weight physically seen by the robot always overrides
    // map-based navigation.
    // ========================================================

    if (roaming_check_for_weight())
    {
        path_reset();
        return;
    }

    if((millis() - PREV_FAN_TIME_MS) > FANNING_PERIOD_MS) {
        perform_fan_operation();
        PREV_FAN_TIME_MS = millis();
        return;
    }


    // ========================================================
    // CURRENT PRIORITY TARGET
    // ========================================================

    float priorityX = 0.0f;
    float priorityY = 0.0f;
    float priorityDistance = 0.0f;
    float priorityHeadingError = 0.0f;

    int candidatePriorityIndex = -1;
    bool hasPriorityTarget =
        getPriorityTargetInfo(
            candidatePriorityIndex,
            priorityX,
            priorityY,
            priorityDistance,
            priorityHeadingError
        );

        bool canUsePriority = hasPriorityTarget;


    // ========================================================
    // CURRENT FRONTIER TARGET
    //
    // This may change as the map changes.
    //
    // It is only used when selecting a NEW goal.
    // While ROAM_DSTAR is active, roamGoalX/Y stay locked.
    // ========================================================

    float frontierX = -1.0f;
    float frontierY = -1.0f;

    bool hasFrontierTarget =
        get_frontier_target(
            frontierX,
            frontierY
        );


    // ========================================================
    // LOCAL CLEARANCES
    // ========================================================

    int outerLeft;
    int innerLeft;
    int upperLeft;
    int innerRight;
    int outerRight;
    int upperRight;

    int front;
    int leftClearance;
    int rightClearance;

    readClearances(
        outerLeft,
        innerLeft,
        upperLeft,
        innerRight,
        outerRight,
        upperRight,
        front,
        leftClearance,
        rightClearance
    );

    Serial.print("OL: "); Serial.print(outerLeft);
    Serial.print(" IL: "); Serial.print(innerLeft);
    Serial.print(" UL: "); Serial.print(upperLeft);
    Serial.print(" IR: "); Serial.print(innerRight);
    Serial.print(" OR: "); Serial.print(outerRight);
    Serial.print(" UR: "); Serial.print(upperRight);
    Serial.print(" F: "); Serial.print(front);
    Serial.print(" LC: "); Serial.print(leftClearance);
    Serial.print(" RC: "); Serial.println(rightClearance);


    printRoamingTelemetry(
        front,
        leftClearance,
        rightClearance,
        hasPriorityTarget,
        hasFrontierTarget
    );


    // ========================================================
    // PRIORITY-TARGET FINAL STATES
    //
    // Do these before critical-obstacle checking because the
    // weight itself may be very close to the front sensors.
    // ========================================================

    if (roamingState ==
        ROAM_TARGET_TURNING)
    {
        if (motor_control_is_turning())
        {
            return;
        }


        motor_control_stop();

        priorityTargetWaitStartedAt =
            millis();

        prioritySearchStage = 0;

        prioritySearchStageAt =
            millis();

        roamingState =
            ROAM_TARGET_WAITING;

        return;
    }


    if (roamingState ==
        ROAM_TARGET_WAITING)
    {
        handlePriorityWaiting();

        return;
    }

    // ========================================================
    // D* STEP TURN
    //
    // A large D* heading change is split into small turns.
    // Once this 20 degree turn finishes, return to ROAM_DSTAR.
    // The next loop will call path_get_lookahead_waypoint()
    // again, which replans using the newly updated map.
    // ========================================================

    if (roamingState ==
        ROAM_DSTAR_STEP_TURNING)
    {
        if (motor_control_is_turning())
        {
            return;
        }

        motor_control_stop();

        debugNav.print("NAV_EVENT,");
        debugNav.print(millis());
        debugNav.println(
            ",DSTAR_STEP_TURN_COMPLETE"
        );

        roamingState =
            ROAM_DSTAR;

        return;
    }

    // ========================================================
    // CRITICAL OBSTACLE SAFETY OVERRIDE
    // ========================================================

    if (checkCriticalObstacle(front))
    {
        path_reset();
        return;
    }


    // ========================================================
    // MAIN ROAMING STATE MACHINE
    // ========================================================

    switch (roamingState)
    {
        // ====================================================
        // SELECT A NEW GOAL
        // ====================================================

        case ROAM_START:
        {
            // -----------------------------------------------
            // Priority targets always beat frontiers.
            // -----------------------------------------------
            
            if (canUsePriority)
            {
                updatePriorityTarget(
                    candidatePriorityIndex,
                    priorityX,
                    priorityY,
                    priorityDistance
                );

                break;
            }


            // -----------------------------------------------
            // No priority target:
            // always explore a frontier.
            // -----------------------------------------------

            if (hasFrontierTarget)
            {
                updateFrontierTarget(
                    frontierX,
                    frontierY
                );

                break;
            }


            // -----------------------------------------------
            // No target and no frontier currently available.
            //
            // Reposition through confirmed free space when the
            // frontier map is temporarily exhausted.
            // -----------------------------------------------

            if (millis() - lastOpenSearchAt >= OPEN_SPACE_RETRY_MS)
            {
                lastOpenSearchAt = millis();
                float openX, openY;
                if (chooseNearbyOpenSpace(openX, openY))
                {
                    debugNav.print("OPEN_SPACE_GOAL,");
                    debugNav.print(openX);
                    debugNav.print(",");
                    debugNav.println(openY);
                    if (startRoamDstar(ROAM_GOAL_OPEN_SPACE, openX, openY))
                        break;
                }
            }

            setRoamGoal(
                ROAM_GOAL_NONE,
                -1.0f,
                -1.0f
            );

            if (motor_control_is_driving() ||
                motor_control_is_driving_to_point() ||
                motor_control_is_turning()) motor_control_stop();

            break;
        }


        // ====================================================
        // FOLLOW CURRENT D* ROUTE
        //
        // This is deliberately structured the same way as
        // HOMING_DSTAR.
        // ====================================================

        case ROAM_DSTAR:
        {
            // -----------------------------------------------
            // A priority target may interrupt frontier
            // exploration.
            //
            // Once a priority target is already locked,
            // changing frontiers are completely ignored.
            // -----------------------------------------------

            if (roamGoal !=
                    ROAM_GOAL_PRIORITY &&
                canUsePriority)
            {
                path_reset();

                motor_control_stop();


                updatePriorityTarget(
                    candidatePriorityIndex,
                    priorityX,
                    priorityY,
                    priorityDistance
                );

                break;
            }

            if (roamGoal == ROAM_GOAL_OPEN_SPACE && hasFrontierTarget)
            {
                updateFrontierTarget(frontierX, frontierY);
                break;
            }


            // -----------------------------------------------
            // If the currently locked priority target no
            // longer exists, choose a new goal.
            // -----------------------------------------------

            if (roamGoal ==
                    ROAM_GOAL_PRIORITY &&
                !hasPriorityTarget)
            {
                path_reset();

                motor_control_stop();


                setRoamGoal(
                    ROAM_GOAL_NONE,
                    -1.0f,
                    -1.0f
                );

                roamingState =
                    ROAM_START;

                break;
            }


            float goalDistance =
                roamGoalDistance();


            // ===============================================
            // PRIORITY TARGET REACHED
            // ===============================================

            if (roamGoal ==
                    ROAM_GOAL_PRIORITY &&
                goalDistance <=
                    PRIORITY_TARGET_ARRIVAL_MM)
            {
                path_reset();

                motor_control_stop();


                float headingError =
                    roamGoalHeadingError();


                // Point directly toward the expected weight
                // before doing the existing sensor sweep.
                if (fabsf(headingError) >
                    PRIORITY_TARGET_FINAL_ALIGN_DEG)
                {
                    debugNav.print(
                        "NAV_EVENT,"
                    );

                    debugNav.print(
                        millis()
                    );

                    debugNav.print(
                        ",PRIORITY_ALIGN,"
                    );

                    debugNav.println(
                        headingError
                    );


                    motor_control_turn_relative(
                        -headingError
                    );


                    roamingState =
                        ROAM_TARGET_TURNING;

                    break;
                }


                debugNav.print(
                    "NAV_EVENT,"
                );

                debugNav.print(
                    millis()
                );

                debugNav.print(
                    ",PRIORITY_REACHED,"
                );

                debugNav.print(
                    roamGoalX
                );

                debugNav.print(",");

                debugNav.println(
                    roamGoalY
                );


                priorityTargetWaitStartedAt =
                    millis();

                prioritySearchStage = 0;

                prioritySearchStageAt =
                    millis();

                roamingState =
                    ROAM_TARGET_WAITING;

                break;
            }


            // ===============================================
            // FRONTIER REACHED
            // ===============================================

            if ((roamGoal == ROAM_GOAL_FRONTIER ||
                 roamGoal == ROAM_GOAL_OPEN_SPACE) &&
                goalDistance <=
                    FRONTIER_TARGET_ARRIVAL_MM)
            {
                if (roamGoal == ROAM_GOAL_OPEN_SPACE)
                {
                    lastOpenGoalX = roamGoalX;
                    lastOpenGoalY = roamGoalY;
                    debugNav.print("OPEN_SPACE_REACHED,");
                    debugNav.print(roamGoalX);
                    debugNav.print(",");
                    debugNav.println(roamGoalY);
                }
                else if (!frontierReachedLogged)
                {
                    frontierReachedLogged =
                        true;


                    debugNav.print(
                        "NAV_EVENT,"
                    );

                    debugNav.print(
                        millis()
                    );

                    debugNav.print(
                        ",FRONTIER_REACHED,"
                    );

                    debugNav.print(
                        roamGoalX
                    );

                    debugNav.print(",");

                    debugNav.println(
                        roamGoalY
                    );
                }

                float nextDx = frontierX - pose_get_x_mm();
                float nextDy = frontierY - pose_get_y_mm();
                bool canChainFrontier = roamGoal == ROAM_GOAL_FRONTIER &&
                    hasFrontierTarget &&
                    hypotf(frontierX - roamGoalX, frontierY - roamGoalY) > 250.0f &&
                    hypotf(nextDx, nextDy) > FRONTIER_TARGET_ARRIVAL_MM + 150.0f &&
                    front > 500 &&
                    fabsf(wrap180(atan2f(nextDy, nextDx) * 180.0f / PI -
                                  pose_get_heading_deg())) <= 30.0f &&
                    !STATE_FLAGS.reverse_triggered &&
                    !STATE_FLAGS.target_identified;
                if (canChainFrontier)
                {
                    debugNav.println("FRONTIER_CHAIN");
                    updateFrontierTarget(frontierX, frontierY, true);
                    break;
                }

                path_reset();
                motor_control_stop();


                // Release this frontier.
                // On the next update the map's latest frontier
                // becomes the new D* goal.
                setRoamGoal(
                    ROAM_GOAL_NONE,
                    -1.0f,
                    -1.0f
                );


                roamingState =
                    ROAM_START;

                break;
            }


            // ===============================================
            // GET D* LOOKAHEAD WAYPOINT
            // ===============================================

            float waypointX;
            float waypointY;


            if (!path_get_lookahead_waypoint(
                    waypointX,
                    waypointY))
            {
                if (roamGoal == ROAM_GOAL_OPEN_SPACE)
                {
                    lastOpenGoalX = roamGoalX;
                    lastOpenGoalY = roamGoalY;
                    debugNav.print("OPEN_SPACE_UNREACHABLE,");
                    debugNav.print(roamGoalX);
                    debugNav.print(",");
                    debugNav.println(roamGoalY);
                }
                // A priority route which USED to work has now
                // become unavailable. Apply the same cooldown
                // before trying that target again.
                if (roamGoal == ROAM_GOAL_PRIORITY)
                {
                    if (priorityTargetIndex >= 0)
                        priorityFailedAt[priorityTargetIndex] = millis();

                    debugNav.print("NAV_EVENT,");
                    debugNav.print(millis());
                    debugNav.println(
                        ",PRIORITY_RETRY_BLOCKED"
                    );
                }

                path_reset();

                motor_control_stop();


                debugNav.print(
                    "NAV_EVENT,"
                );

                debugNav.print(
                    millis()
                );

                debugNav.print(
                    ",ROAM_DSTAR_UNAVAILABLE,"
                );

                debugNav.println(
                    roamGoalName()
                );


                // There is no blind fallback.
                // Release the failed goal and let ROAM_START
                // select the best currently available target.
                setRoamGoal(
                    ROAM_GOAL_NONE,
                    -1.0f,
                    -1.0f
                );


                roamingState =
                    ROAM_START;

                break;
            }


            // ===============================================
            // DRIVE TOWARD D* LOOKAHEAD
            // ===============================================

            // Work out the heading from the robot's current
            // pose to the D* waypoint.
            float dx =
                waypointX -
                pose_get_x_mm();

            float dy =
                waypointY -
                pose_get_y_mm();


            float desiredHeading =
                atan2f(dy, dx) *
                180.0f / PI;

            if (desiredHeading < 0.0f)
            {
                desiredHeading += 360.0f;
            }


            // Heading error in the POSE coordinate frame.
            float waypointHeadingError =
                wrap180(
                    desiredHeading -
                    pose_get_heading_deg()
                );


            // ------------------------------------------------
            // LARGE D* TURN
            //
            // Do not immediately point-turn 60, 70, 90 deg.
            //
            // Turn at most 20 degrees, then allow D* to
            // reconsider the route using the newly updated map.
            // ------------------------------------------------

            if (fabsf(waypointHeadingError) >
                DSTAR_LARGE_TURN_DEG)
            {
                motor_control_stop();


                float turnStep =
                    constrain(
                        waypointHeadingError,
                        -DSTAR_TURN_STEP_DEG,
                        DSTAR_TURN_STEP_DEG
                    );


                debugNav.print(
                    "NAV_EVENT,"
                );

                debugNav.print(
                    millis()
                );

                debugNav.print(
                    ",DSTAR_STEP_TURN,"
                );

                debugNav.print(
                    waypointHeadingError
                );

                debugNav.print(",");

                debugNav.println(
                    turnStep
                );


                // IMPORTANT:
                // pose heading and raw IMU heading increase in
                // opposite directions in the current robot setup.
                //
                // Therefore negate the pose-frame turn.
                motor_control_turn_relative_coarse(
                    -turnStep
                );


                roamingState =
                    ROAM_DSTAR_STEP_TURNING;


                break;
            }


            // ------------------------------------------------
            // Normal D* driving when reasonably aligned.
            // ------------------------------------------------

            int power =
                goalDistance <
                    ROAM_DSTAR_SLOW_DISTANCE_MM
                    ? ROAM_SLOW_POWER
                    : ROAM_POWER;


            motor_control_drive_to_point(
                waypointX,
                waypointY,
                power
            );


            break;
        }


        // These are handled before the switch so they cannot
        // fall through into normal D* navigation.
        case ROAM_DSTAR_STEP_TURNING:
        case ROAM_TARGET_TURNING:
        case ROAM_TARGET_WAITING:
        {
            break;
        }


        default:
        {
            path_reset();

            motor_control_stop();

            setRoamGoal(
                ROAM_GOAL_NONE,
                -1.0f,
                -1.0f
            );

            roamingState =
                ROAM_START;

            break;
        }
    }
}
