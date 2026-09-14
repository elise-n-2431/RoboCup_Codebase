#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "inputs/tof_expander.h"
#include "state_machine.h"
#include "map.h"



static const int WEIGHT_DETECT_DISTANCE_MM = 550;

static int middleLostCount = 0;
static const int MIDDLE_LOST_COUNT_REQUIRED = 3;


static const float LEFT_WEIGHT_TURN_DEG  = -60.0f;
static const float RIGHT_WEIGHT_TURN_DEG = 60.0f;
static const int WEIGHT_DIFFERENCE_MM = 100;
static const int WEIGHT_STOP_DISTANCE_MM = 55;
static int DETECTION_COUNT_REQUIRED = 3;

static const int WEIGHT_SLOW_DISTANCE_MM = 150;

static const int WEIGHT_APPROACH_POWER = 320;
static const int WEIGHT_SLOW_POWER = 260;


static int leftDetectionCount = 0;
static int rightDetectionCount = 0;
static float weightApproachHeading = 0.0f;
static bool weightApproachSlowed = false;


//In the roaming state to see which side it thinks the weight is on
enum WeightTargetSide
{
    TARGET_NONE,
    TARGET_LEFT,
    TARGET_RIGHT
};

static WeightTargetSide weightTargetSide = TARGET_NONE;


enum PursuitState
{
    PURSUIT_START,
    PURSUIT_TURNING,
    PURSUIT_ACQUIRING,
    PURSUIT_APPROACHING,
    PURSUIT_FINISHED
};

static PursuitState pursuitState = PURSUIT_START;

enum ReversingState
{
    REVERSE_START,
    REVERSE_BACKING,
    REVERSE_TURNING,
    REVERSE_FINISHED
};

static ReversingState reversingState = REVERSE_START;

enum RoamingState
{
    ROAM_START,
    ROAM_DRIVING,
    ROAM_TURNING,
    ROAM_CHECKING
};
static RoamingState roamingState = ROAM_START;


// ============================================================
// ROAMING TUNING
// ============================================================

static int ROAM_POWER = 380;

// Distance at which normal avoidance begins
static int ROAM_FRONT_BLOCK_MM = 250;

static int ROAM_CRITICAL_MM = 90;

// Normal avoidance turn
static float ROAM_AVOID_TURN_DEG = 45.0f;

// More aggressive recovery turn later if needed
static float ROAM_RECOVERY_TURN_DEG = 90.0f;

static int ROAM_REVERSE_POWER = 250;
static unsigned long ROAM_REVERSE_TIME_MS = 500;

static const int REVERSE_POWER = 250;
static const unsigned long REVERSE_TIME_MS = 2000;
//to time when the reverse started so it knows when to count 1 second from
static unsigned long reverseStartedAt = 0;

static const int ROAM_SIDE_BLOCK_MM = 180;
static const int ROAM_SLOW_MM = 500;
static const int ROAM_SLOW_POWER = 280;

static const unsigned long NAV_TURN_TIMEOUT_MS = 4000;
static const unsigned long PURSUIT_TIMEOUT_MS = 10000;

static bool navigatorEnabled = false;
static bool roamingPickupEnabled = false;
static bool turnWatchActive = false;

static NavState lastNavState = STATIONARY;

static unsigned long navigatorStateStarted = 0;
static unsigned long turnStartedAt = 0;
static unsigned long roamCheckStartedAt = 0;
static unsigned long lastWeightCheckAt = 0;

static float roamHeading = 0.0f;
static int roamCommandedPower = 0;
static int roamTurnDirection = 1;






static bool weightPairDetected(int top, int bottom, int &difference)
{
    difference = 0;

    if (top < 0 || bottom <= 0 || bottom > WEIGHT_DETECT_DISTANCE_MM) return false;
    if (top == 0) return true;

    difference = top - bottom;
    return difference >= WEIGHT_DIFFERENCE_MM;
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

static bool obstacleCloserThan(int distance,int threshold)
{
    return (distance > 0 && distance < threshold);
}

//for now it just has the ability to  look for weights and riase flags
static void detect_weights_exe()
{
    static uint32_t lastLeftTop = 0, lastLeftBottom = 0;
    static uint32_t lastRightTop = 0, lastRightBottom = 0;

    int leftTop = tof_get_weight_left_top();
    int leftBottom = tof_get_weight_left_bottom();
    int rightTop = tof_get_weight_right_top();
    int rightBottom = tof_get_weight_right_bottom();

    uint32_t leftTopSample = tof_get_sample_number(4);
    uint32_t leftBottomSample = tof_get_sample_number(3);
    uint32_t rightTopSample = tof_get_sample_number(2);
    uint32_t rightBottomSample = tof_get_sample_number(1);

    if (leftTop < 0 || leftBottom < 0) leftDetectionCount = 0;
    if (rightTop < 0 || rightBottom < 0) rightDetectionCount = 0;

    int difference = 0;

    if (leftTopSample != lastLeftTop && leftBottomSample != lastLeftBottom) {
        lastLeftTop = leftTopSample;
        lastLeftBottom = leftBottomSample;

        if (weightPairDetected(leftTop, leftBottom, difference)) leftDetectionCount++;
        else leftDetectionCount = 0;
    }

    if (rightTopSample != lastRightTop && rightBottomSample != lastRightBottom) {
        lastRightTop = rightTopSample;
        lastRightBottom = rightBottomSample;

        if (weightPairDetected(rightTop, rightBottom, difference)) rightDetectionCount++;
        else rightDetectionCount = 0;
    }

    if (leftDetectionCount >= DETECTION_COUNT_REQUIRED) {
        weightTargetSide = TARGET_LEFT;
        Serial2.println("Roaming: weight detected LEFT");
    } else if (rightDetectionCount >= DETECTION_COUNT_REQUIRED) {
        weightTargetSide = TARGET_RIGHT;
        Serial2.println("Roaming: weight detected RIGHT");
    } else return;

    pursuitState = PURSUIT_START;
    leftDetectionCount = rightDetectionCount = 0;

    motor_control_stop();
    setStateFlag(&STATE_FLAGS.target_identified);
}

void navigator_stop()
{
    navigatorEnabled = false;
    motor_control_stop();

    turnWatchActive = false;
    roamingState = ROAM_START;
    leftDetectionCount = rightDetectionCount = 0;

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
        Serial2.println("Navigator: start from ROAMING or STATIONARY");
        return false;
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) return false;

    navigator_stop();

    navigatorEnabled = true;
    roamingPickupEnabled = enablePickup;
    lastNavState = getNavState();

    navigatorStateStarted = millis();
    lastWeightCheckAt = millis();
    roamCommandedPower = 0;
    weightTargetSide = TARGET_NONE;

    Serial2.println(enablePickup ? "Navigator: roaming + pickup" : "Navigator: roaming only");
    return true;
}

static void roaming_start_turn(int leftClearance, int rightClearance)
{
    if (abs(leftClearance - rightClearance) > 80) {
        roamTurnDirection = leftClearance > rightClearance ? -1 : 1;
    }

    motor_control_stop();
    motor_control_turn_relative(roamTurnDirection * ROAM_AVOID_TURN_DEG);

    roamingState = ROAM_TURNING;
    roamCommandedPower = 0;
    leftDetectionCount = rightDetectionCount = 0;

    Serial2.println(roamTurnDirection < 0 ? "Roaming: turn LEFT" : "Roaming: turn RIGHT");
}

static void roaming_drive(int power)
{
    if (motor_control_is_driving() && roamCommandedPower == power) return;

    motor_control_drive_heading(roamHeading, power);
    roamCommandedPower = power;
}



static void roaming_exe()
{
    if (roamingState == ROAM_TURNING)
    {
        if (motor_control_is_turning()) return;

        roamCheckStartedAt = millis();
        roamingState = ROAM_CHECKING;
        return;
    }

    int outerLeft = tof_get_nav_outer_left();
    int innerLeft = tof_get_nav_inner_left();
    int innerRight = tof_get_nav_inner_right();
    int outerRight = tof_get_nav_outer_right();

    static unsigned long navTofInvalidStarted = 0;

    /*bool tofUnavailable =
        outerLeft < 0 ||
        innerLeft < 0 ||
        innerRight < 0 ||
        outerRight < 0;

    if (tofUnavailable)
    {
        if (navTofInvalidStarted == 0)
        {
            navTofInvalidStarted = millis();
        }

        motor_control_stop();

        if (millis() - navTofInvalidStarted > 500)
        {
            navigator_stop();
            Serial2.println("Roaming stopped: navigation ToF unavailable");
        }

        return;
    }

    navTofInvalidStarted = 0;*/

    bool weakReturn = outerLeft == 0 || innerLeft == 0 || innerRight == 0 || outerRight == 0;

    outerLeft = clearanceValue(outerLeft);
    innerLeft = clearanceValue(innerLeft);
    innerRight = clearanceValue(innerRight);
    outerRight = clearanceValue(outerRight);

    int front = min(innerLeft, innerRight);
    int leftClearance = min(outerLeft, innerLeft);
    int rightClearance = min(outerRight, innerRight);

    if (front <= ROAM_CRITICAL_MM) {
        navigator_stop();
        Serial2.println("Roaming stopped: obstacle critically close");
        return;
    }

    switch (roamingState) {
        case ROAM_START:
            motor_control_stop();
            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;
            break;

        case ROAM_TURNING:
            if (motor_control_is_turning()) return;

            roamCheckStartedAt = millis();
            roamingState = ROAM_CHECKING;
            break;

        case ROAM_CHECKING:
            if (millis() - roamCheckStartedAt < 150) return;

            if (front < ROAM_FRONT_BLOCK_MM + 80 ||
                outerLeft < ROAM_SIDE_BLOCK_MM + 30 ||
                outerRight < ROAM_SIDE_BLOCK_MM + 30) {
                roaming_start_turn(leftClearance, rightClearance);
                return;
            }

            roamHeading = imu_get_heading();
            roamCommandedPower = 0;
            roamingState = ROAM_DRIVING;

            roaming_drive(front < ROAM_SLOW_MM ? ROAM_SLOW_POWER : ROAM_POWER);
            break;

        case ROAM_DRIVING:
            if (front < ROAM_FRONT_BLOCK_MM ||
                outerLeft < ROAM_SIDE_BLOCK_MM ||
                outerRight < ROAM_SIDE_BLOCK_MM) {
                roaming_start_turn(leftClearance, rightClearance);
                return;
            }

            if (roamingPickupEnabled &&
                millis() - navigatorStateStarted >= 1000 &&
                millis() - lastWeightCheckAt >= 100) {
                lastWeightCheckAt = millis();
                detect_weights_exe();

                if (STATE_FLAGS.target_identified) return;
            }

            roaming_drive(front < ROAM_SLOW_MM ? ROAM_SLOW_POWER : ROAM_POWER);
            break;
    }
}



static void pursuit_exe()
{
    switch (pursuitState)
    {
        case PURSUIT_START:
        {
            if (weightTargetSide == TARGET_LEFT)
            {
                Serial2.println("Pursuit: turning LEFT toward weight");
                motor_control_turn_relative(LEFT_WEIGHT_TURN_DEG);
                pursuitState = PURSUIT_TURNING;
            }
            else if (weightTargetSide == TARGET_RIGHT)
            {
                Serial2.println("Pursuit: turning RIGHT toward weight");
                motor_control_turn_relative(RIGHT_WEIGHT_TURN_DEG);
                pursuitState = PURSUIT_TURNING;
            }
            else
            {
                Serial2.println("Pursuit started without target side");
            }

            break;
        }

        case PURSUIT_TURNING:
        {
            if (motor_control_is_turning()) return;

            Serial2.println("Pursuit: turn complete");
            pursuitState = PURSUIT_ACQUIRING;

            break;
        }

        case PURSUIT_ACQUIRING:
        {
            int centreDistance = tof_get_weight_middle();

            if (centreDistance > 0 && centreDistance <= WEIGHT_DETECT_DISTANCE_MM)
            {
                Serial2.print("Pursuit: centre acquired weight at ");
                Serial2.print(centreDistance);
                Serial2.println(" mm");

                middleLostCount = 0;
                weightApproachHeading = imu_get_heading();
                weightApproachSlowed = false;

                pursuitState = PURSUIT_APPROACHING;
                return;
            }

            break;
        }

        case PURSUIT_APPROACHING:
        {
            int centreDistance = tof_get_weight_middle();

            // Lost target
            if (centreDistance <= 0 || centreDistance > WEIGHT_DETECT_DISTANCE_MM)
            {
                motor_control_stop();
                weightApproachSlowed = false;
                middleLostCount++;

                if (middleLostCount >= MIDDLE_LOST_COUNT_REQUIRED)
                {
                    Serial2.println("Pursuit: centre lost weight");
                    middleLostCount = 0;
                    pursuitState = PURSUIT_ACQUIRING;
                }

                return;
            }

            middleLostCount = 0;

            // Weight has reached the entrance
            if (centreDistance <= WEIGHT_STOP_DISTANCE_MM)
            {
                motor_control_stop();

                Serial2.print("Pursuit: weight reached at ");
                Serial2.print(centreDistance);
                Serial2.println(" mm");

                pursuitState = PURSUIT_FINISHED;
                setStateFlag(&STATE_FLAGS.weight_in_entrance);

                return;
            }

            // Slow down near the weight
            if (centreDistance <= WEIGHT_SLOW_DISTANCE_MM)
            {
                if (!weightApproachSlowed || !motor_control_is_driving())
                {
                    motor_control_drive_heading(weightApproachHeading, WEIGHT_SLOW_POWER);
                    weightApproachSlowed = true;

                    Serial2.println("Pursuit: slowing approach");
                }

                return;
            }

            // Start normal approach
            if (!motor_control_is_driving())
            {
                motor_control_drive_heading(weightApproachHeading, WEIGHT_APPROACH_POWER);
            }

            break;
        }

        case PURSUIT_FINISHED:
        {
            // Wait for top-level state machine to move PURSUIT -> SORTING
            break;
        }
    }
}

static void reversing_exe()
{
    switch (reversingState)
    {
        case REVERSE_START:
        {
            Serial.println("Reverse: backing away");

            motor_control_reverse(REVERSE_POWER);
            reverseStartedAt = millis();

            reversingState = REVERSE_BACKING;
            break;
        }

        case REVERSE_BACKING:
        {
            if (millis() - reverseStartedAt < REVERSE_TIME_MS) return;

            motor_control_stop();

            if (weightTargetSide == TARGET_LEFT)
            {
                Serial.println("Reverse: turning RIGHT");
                motor_control_turn_relative(-LEFT_WEIGHT_TURN_DEG);
            }
            else if (weightTargetSide == TARGET_RIGHT)
            {
                Serial.println("Reverse: turning LEFT");
                motor_control_turn_relative(-RIGHT_WEIGHT_TURN_DEG);
            }
            else
            {
                Serial.println("Reverse: no target side, skipping turn");
                reversingState = REVERSE_FINISHED;
                return;
            }

            reversingState = REVERSE_TURNING;
            break;
        }

        case REVERSE_TURNING:
        {
            if (motor_control_is_turning()) return;

            Serial.println("Reverse: turn complete");
            reversingState = REVERSE_FINISHED;
            break;
        }

        case REVERSE_FINISHED:
        {
            Serial.println("Reverse: manoeuvre complete");

            weightTargetSide = TARGET_NONE;
            reversingState = REVERSE_START;

            setStateFlag(&STATE_FLAGS.reverse_complete);
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

        leftDetectionCount = rightDetectionCount = middleLostCount = 0;

        if (nav == ROAMING) {
            roamingState = ROAM_START;
            weightTargetSide = TARGET_NONE;
            roamCommandedPower = 0;
        }

        if (nav == PURSUIT) {
            pursuitState = PURSUIT_START;
            weightApproachSlowed = false;
        }

        if (nav == REVERSING) {
            reversingState = REVERSE_START;
        }
    }

    if (!imu_is_online() || !isfinite(imu_get_heading())) {
        navigator_stop();
        Serial2.println("Navigator stopped: IMU unavailable");
        return;
    }

    if (!motor_control_is_turning()) turnWatchActive = false;

    if (turnWatchActive && millis() - turnStartedAt >= NAV_TURN_TIMEOUT_MS) {
        navigator_stop();
        Serial2.println("Navigator stopped: turn timeout");
        return;
    }

    if (nav == PURSUIT) {
        int left = tof_get_nav_inner_left();
        int right = tof_get_nav_inner_right();

        if (millis() - navigatorStateStarted >= PURSUIT_TIMEOUT_MS)
        {
            motor_control_stop();
            setStateFlag(&STATE_FLAGS.target_lost);
            return;
        }
    }

    switch (nav) {
        case ROAMING:
            roaming_exe();
            break;

        case PURSUIT:
            pursuit_exe();
            break;

        case REVERSING:
            reversing_exe();
            break;

        case HOMING:
            navigator_stop();
            Serial2.println("Navigator stopped: homing not implemented");
            break;

        default:
            break;
    }

    if (motor_control_is_turning() && !turnWatchActive) {
        turnStartedAt = millis();
        turnWatchActive = true;
    }
}

