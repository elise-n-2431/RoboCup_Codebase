#include "reversing_controller.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "debug_print.h"
#include "navigation/weight_detection.h"


// Reverse tuning constants

static const int REVERSE_POWER = 250;
static const unsigned long REVERSE_TIME_MS = 2000;

static const int CRITICAL_REVERSE_POWER = 250;
static const unsigned long CRITICAL_REVERSE_TIME_MS = 500;

static const float REVERSE_ESCAPE_TURN_DEG = 60.0f;
static const float CRITICAL_ESCAPE_TURN_DEG = 90.0f;

static const unsigned long WEIGHT_RETRIGGER_BLOCK_MS = 1500;


// Ramp detection
static const float RAMP_TRIGGER_DEG = 10.0f;
static const float RAMP_RELEASE_DEG = 3.0f;

static const unsigned long RAMP_CONFIRM_MS = 200;
static const unsigned long RAMP_MIN_REVERSE_MS = 600;
static const unsigned long RAMP_MAX_REVERSE_MS = 2500;
static const unsigned long RAMP_RETRIGGER_BLOCK_MS = 1500;

enum ReversingState
{
    REVERSE_START,
    REVERSE_BACKING,
    REVERSE_TURNING,
    REVERSE_FINISHED
};

static ReversingState reversingState = REVERSE_START;
static ReverseReason pendingReason = REVERSE_NORMAL;
static ReverseReason activeReason = REVERSE_NORMAL;

static bool reasonPending = false;
static unsigned long reverseStartedAt = 0;

static float flatPitchReference = 0.0f;
static unsigned long pitchExceededAt = 0;
static unsigned long rampDetectionBlockedUntil = 0;
static int fallbackTurnDirection = 1;


static int reverseClearanceValue(int distance)
{
    // 0 means no obstacle return, so treat as large clearance.
    if (distance == 0) return 1200;

    return distance;
}


static int reverseSideClearance(int outer, int inner)
{
    // Both readings unusable.
    if (outer < 0 && inner < 0)
    {
        return -1;
    }

    int outerClear = outer < 0 ? -1 : reverseClearanceValue(outer);
    int innerClear = inner < 0 ? -1 : reverseClearanceValue(inner);

    if (outerClear < 0) return innerClear;
    if (innerClear < 0) return outerClear;
    return min(outerClear, innerClear);
}

//need different behaviour for reversing depending on what caused it
void reversing_set_reason(ReverseReason reason)
{
    pendingReason = reason;
    reasonPending = true;
}

void reversing_set_flat_pitch_reference(float pitch)
{
    flatPitchReference = pitch;
    pitchExceededAt = 0;
    rampDetectionBlockedUntil = 0;
}

void reversing_start()
{
    activeReason = reasonPending ? pendingReason : REVERSE_NORMAL;

    pendingReason = REVERSE_NORMAL;
    reasonPending = false;

    reversingState = REVERSE_START;
    reverseStartedAt = 0;
}

bool reversing_check_for_pitch_escape(NavState nav, bool suppressCheck)
{
    unsigned long now = millis();
    // Only relevant while actually navigating.
    if (nav != ROAMING && nav != PURSUIT && nav != HOMING)
    {
        pitchExceededAt = 0;
        return false;
    }

    // Used during final homing/docking.
    if (suppressCheck)
    {
        pitchExceededAt = 0;
        return false;
    }

    if ((int32_t)(now - rampDetectionBlockedUntil) < 0)
    {
        pitchExceededAt = 0;
        return false;
    }

    float pitch = imu_get_pitch();
    float pitchDifference = fabsf(pitch - flatPitchReference);

    if (pitchDifference < RAMP_TRIGGER_DEG)
    {
        pitchExceededAt = 0;
        return false;
    }

    // First abnormal reading.
    if (pitchExceededAt == 0)
    {
        pitchExceededAt = now;
        return false;
    }

    // Require sustained pitch rather than one bad sample.
    if (now - pitchExceededAt < RAMP_CONFIRM_MS)
    {
        return false;
    }

    motor_control_stop();
    debugNav.print("PITCH ESCAPE: pitch=");
    debugNav.print(pitch);
    debugNav.print(" reference=");
    debugNav.print(flatPitchReference);
    debugNav.print(" difference=");
    debugNav.println(pitchDifference);
    reversing_set_reason(REVERSE_RAMP);
    pitchExceededAt = 0;
    // Block another trigger while transitioning into reverse.
    rampDetectionBlockedUntil = now + 3000;

    if (nav == PURSUIT)
    {
        setStateFlag(&STATE_FLAGS.target_lost);
    }

    setStateFlag(&STATE_FLAGS.reverse_triggered);

    return true;
}

void reversing_update()
{
    switch (reversingState)
    {
        case REVERSE_START:
        {
            motor_control_stop();

            if (activeReason == REVERSE_RAMP)
            {
                debugNav.println("Reverse: ramp escape - backing away");
            }
            else if (activeReason == REVERSE_CRITICAL_OBSTACLE)
            {
                debugNav.println("Reverse: critical obstacle escape - backing away");
            }
            else
            {
                debugNav.println("Reverse: backing away");
            }

            int reversePower =
                activeReason == REVERSE_CRITICAL_OBSTACLE
                ? CRITICAL_REVERSE_POWER
                : REVERSE_POWER;

            motor_control_reverse(reversePower);

            reverseStartedAt = millis();
            reversingState = REVERSE_BACKING;

            break;
        }


        case REVERSE_BACKING:
        {
            unsigned long elapsed = millis() - reverseStartedAt;

            // Ramp escape:
            // reverse for at least the minimum time, then continue
            // until level again or the maximum time is reached.
            if (activeReason == REVERSE_RAMP)
            {
                float pitchError = fabsf(imu_get_pitch() - flatPitchReference);

                if (elapsed < RAMP_MIN_REVERSE_MS)
                {
                    return;
                }

                if (pitchError > RAMP_RELEASE_DEG &&
                    elapsed < RAMP_MAX_REVERSE_MS)
                {
                    return;
                }
            }

            // Critical wall only needs a short separation.
            else if (activeReason == REVERSE_CRITICAL_OBSTACLE)
            {
                if (elapsed < CRITICAL_REVERSE_TIME_MS)
                {
                    return;
                }
            }

            // Dummy / failed collection.
            else
            {
                if (elapsed < REVERSE_TIME_MS)
                {
                    return;
                }
            }
            motor_control_stop();


            // ------------------------------------------------
            // Choose clearer side
            // ------------------------------------------------
            int leftClearance = reverseSideClearance(tof_get_nav_outer_left(), tof_get_nav_inner_left());

            int rightClearance = reverseSideClearance(tof_get_nav_outer_right(),tof_get_nav_inner_right());
            int turnDirection = 0;

            // -1 = LEFT
            // +1 = RIGHT

            if (leftClearance >= 0 && rightClearance >= 0)
            {
                if (leftClearance > rightClearance)
                {
                    turnDirection = -1;
                    debugNav.println("Reverse: escape turn LEFT");
                }
                else
                {
                    turnDirection = 1;
                    debugNav.println("Reverse: escape turn RIGHT");
                }
            }
            else if (leftClearance >= 0)
            {
                turnDirection = -1;
                debugNav.println("Reverse: only LEFT clearance known");
            }
            else if (rightClearance >= 0)
            {
                turnDirection = 1;
                debugNav.println("Reverse: only RIGHT clearance known");
            }
            else
            {
                turnDirection = fallbackTurnDirection;
                fallbackTurnDirection *= -1;

                debugNav.println("Reverse: clearance unknown - fallback turn");
            }

            debugNav.print("Reverse: left clearance=");
            debugNav.print(leftClearance);
            debugNav.print(" right clearance=");
            debugNav.println(rightClearance);


            float escapeTurnAngle =
                activeReason == REVERSE_CRITICAL_OBSTACLE
                ? CRITICAL_ESCAPE_TURN_DEG
                : REVERSE_ESCAPE_TURN_DEG;

            debugNav.print("Reverse: escape turn angle=");
            debugNav.println(turnDirection * escapeTurnAngle);

            motor_control_turn_relative(turnDirection * escapeTurnAngle);

            reversingState = REVERSE_TURNING;
            break;
        }
        case REVERSE_TURNING:
        {
            if (motor_control_is_turning())
            {
                return;
            }

            motor_control_stop();

            debugNav.println("Reverse: escape turn complete");

            reversingState = REVERSE_FINISHED;
            break;
        }
        case REVERSE_FINISHED:
        {
            debugNav.println("Reverse: manoeuvre complete");

            // Don't immediately rediscover the same rejected object.
            weight_detection_block_for(WEIGHT_RETRIGGER_BLOCK_MS);

            // Don't immediately trigger another ramp escape.
            rampDetectionBlockedUntil = millis() + RAMP_RETRIGGER_BLOCK_MS;

            activeReason = REVERSE_NORMAL;
            reversingState = REVERSE_START;

            setStateFlag(&STATE_FLAGS.reverse_complete);

            break;
        }
    }
}