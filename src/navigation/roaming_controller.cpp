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
#include "path_finding.h"

#include "navigation/weight_detection.h"
#include "navigation/pursuit_controller.h"
#include "navigation/reversing_controller.h"
#include "navigation/rejected_weights.h"


enum RoamingState
{
    ROAM_START,
    ROAM_DSTAR,
    ROAM_TARGET_TURNING,
    ROAM_TARGET_WAITING
};

enum RoamGoalType
{
    ROAM_GOAL_NONE,
    ROAM_GOAL_PRIORITY,
    ROAM_GOAL_FRONTIER
};

static RoamingState roamingState =
    ROAM_START;

static RoamGoalType roamGoal =
    ROAM_GOAL_NONE;

static const int ROAM_POWER = 430;
static const int ROAM_SLOW_POWER = 340;

static const float ROAM_SLOW_DISTANCE_MM = 700.0f;
static const int ROAM_SLOW_MM = 250;
static const int ROAM_CRITICAL_MM = 90;

static const float FRONTIER_TARGET_ARRIVAL_MM = 120.0f;

static const float ROAM_DSTAR_SLOW_DISTANCE_MM = 500.0f;


static const float PRIORITY_TARGET_ARRIVAL_MM = 80.0f;
static const float PRIORITY_TARGET_FINAL_ALIGN_DEG = 8.0f;

static const unsigned long PRIORITY_TARGET_CONFIRM_MS = 1200;

// If D* cannot currently reach a priority target,
// don't hammer the same failed target every loop.
// Explore a frontier for a while and try again later.
static const unsigned long PRIORITY_RETRY_DELAY_MS = 5000;

static unsigned long priorityFailedAt = 0;
static bool priorityRetryBlocked = false;

static const unsigned long NAV_TELEMETRY_PERIOD_MS = 200;

static unsigned long lastNavTelemetryAt = 0;

static const float PRIORITY_SEARCH_ANGLE_DEG = 12.0f;
static const unsigned long PRIORITY_SEARCH_SETTLE_MS = 200;

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
        case ROAM_START:
            return "START";

        case ROAM_DSTAR:
            return "DSTAR";

        case ROAM_TARGET_TURNING:
            return "TARGET_TURNING";

        case ROAM_TARGET_WAITING:
            return "TARGET_WAITING";

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

static bool startRoamDstar(
    RoamGoalType goalType,
    float goalX,
    float goalY)
{
    motor_control_stop();

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
        // Don't immediately retry an unreachable priority
        // hundreds of times per second.
        if (goalType == ROAM_GOAL_PRIORITY)
        {
            priorityFailedAt = millis();
            priorityRetryBlocked = true;

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
        if (goalType == ROAM_GOAL_PRIORITY)
        {
            priorityRetryBlocked = false;
        }


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
        roamGoalX -
        pose_get_x_mm();

    float dy =
        roamGoalY -
        pose_get_y_mm();

    float desiredHeading =
        atan2f(dy, dx) *
        180.0f / PI;

    return wrap180(
        desiredHeading -
        pose_get_heading_deg()
    );
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

static bool handlePriorityWaiting(
    bool hasPriorityTarget,
    float priorityX,
    float priorityY)
{
    if (roamingState !=
        ROAM_TARGET_WAITING)
    {
        return false;
    }

    if (!hasPriorityTarget)
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

    priority_targets_remove(0);

    priorityTargetWaitStartedAt = 0;
    prioritySearchStage = 0;

    roamCommandedPower = 0;
    roamCommandedHeading = NAN;

    roamingState = ROAM_START;

    return true;
}


static void updatePriorityTarget(
    float priorityX,
    float priorityY)
{
    startRoamDstar(
        ROAM_GOAL_PRIORITY,
        priorityX,
        priorityY
    );
}

static void updateFrontierTarget(
    float frontierX,
    float frontierY)
{
    frontierReachedLogged =
        false;

    startRoamDstar(
        ROAM_GOAL_FRONTIER,
        frontierX,
        frontierY
    );
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
    path_reset();

    roamingState =
        ROAM_START;

    roamGoal =
        ROAM_GOAL_NONE;


    roamingStartedAt =
        millis();

    lastWeightCheckAt =
        millis();

    priorityFailedAt = 0;
    priorityRetryBlocked = false;
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

    if (checkForWeight())
    {
        path_reset();
        return;
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

        // Release the cooldown once enough time has elapsed.
        if (priorityRetryBlocked &&
            millis() - priorityFailedAt >=
                PRIORITY_RETRY_DELAY_MS)
        {
            priorityRetryBlocked = false;

            debugNav.print("NAV_EVENT,");
            debugNav.print(millis());
            debugNav.println(",PRIORITY_RETRY_READY");
        }


        bool canUsePriority =
            hasPriorityTarget &&
            !priorityRetryBlocked;


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
        handlePriorityWaiting(
            hasPriorityTarget,
            priorityX,
            priorityY
        );

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
            motor_control_stop();


            // -----------------------------------------------
            // Priority targets always beat frontiers.
            // -----------------------------------------------
            
            if (canUsePriority)
            {
                updatePriorityTarget(
                    priorityX,
                    priorityY
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
            // There is intentionally NO blind fallback roam.
            // Stay stopped until mapping produces a frontier.
            // -----------------------------------------------

            setRoamGoal(
                ROAM_GOAL_NONE,
                -1.0f,
                -1.0f
            );

            motor_control_stop();

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

            if (roamGoal ==
                    ROAM_GOAL_FRONTIER &&
                canUsePriority)
            {
                path_reset();

                motor_control_stop();


                updatePriorityTarget(
                    priorityX,
                    priorityY
                );

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

            if (roamGoal ==
                    ROAM_GOAL_FRONTIER &&
                goalDistance <=
                    FRONTIER_TARGET_ARRIVAL_MM)
            {
                path_reset();

                motor_control_stop();


                if (!frontierReachedLogged)
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
                // A priority route which USED to work has now
                // become unavailable. Apply the same cooldown
                // before trying that target again.
                if (roamGoal == ROAM_GOAL_PRIORITY)
                {
                    priorityFailedAt = millis();
                    priorityRetryBlocked = true;

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