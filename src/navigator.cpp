#include "navigator.h"

#include <Arduino.h>
#include <math.h>

#include "inputs/imu.h"
#include "driving_controller.h"
#include "state_machine.h"
#include "debug_print.h"

#include "navigation/weight_detection.h"
#include "navigation/roaming_controller.h"
#include "navigation/pursuit_controller.h"
#include "navigation/reversing_controller.h"
#include "navigation/homing_controller.h"
#include "navigation/rejected_weights.h"

#include "map.h"
#include "pose.h"
#include "inputs/tof_expander.h"
#include "outputs/DC_motors.h"
#include "inputs/encoders.h"

// ============================================================
// NAVIGATOR STATE
// ============================================================

static bool navigatorEnabled = true;
static bool turnWatchActive = false;

static NavState lastNavState = STATIONARY;

static unsigned long turnStartedAt = 0;

static const unsigned long NAV_TURN_TIMEOUT_MS = 8000;

static const unsigned long
    NAV_STUCK_TIMEOUT_MS = 20000;

static unsigned long
    lastPhysicalMotionAt = 0;

static float
    lastPhysicalLeftMm = 0.0f;

static float
    lastPhysicalRightMm = 0.0f;

static float
    lastPhysicalHeading = 0.0f;

static NavState
    motionWatchState = STATIONARY;


// ============================================================
// HELPERS
// ============================================================

static void printNavStateEvent(NavState nav)
{
    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",STATE,");
    debugNav.println(
        getNavStateName()
    );
}


// ============================================================
// INITIALISATION
// ============================================================

void navigator_init()
{
    navigator_stop();
}


// ============================================================
// STOP / START
// ============================================================

void navigator_stop()
{
    navigatorEnabled = false;

    motor_control_stop();

    turnWatchActive = false;

    roaming_reset();
    weight_detection_reset();

    if (getNavState() == ROAMING)
    {
        resetStateFlag(
            &STATE_FLAGS.target_identified
        );
    }

    if (getNavState() == PURSUIT)
    {
        setStateFlag(
            &STATE_FLAGS.target_lost
        );
    }

    if (getNavState() == REVERSING)
    {
        setStateFlag(
            &STATE_FLAGS.reverse_complete
        );
    }
}


bool navigator_start(bool enablePickup)
{
    if (getCollectState() != IDLE ||
        (getNavState() != ROAMING &&
         getNavState() != STATIONARY))
    {
        return false;
    }

    if (!imu_is_online() ||
        !isfinite(imu_get_heading()))
    {
        return false;
    }

    float flatPitchReference =
        imu_get_pitch();

    reversing_set_flat_pitch_reference(
        flatPitchReference
    );

    navigator_stop();
    rejected_weights_reset();

    navigatorEnabled = true;
    lastNavState = getNavState();

    turnWatchActive = false;

    pursuit_reset();

    roaming_start(
        enablePickup
    );

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",NAVIGATOR_START,");
    debugNav.println(
        enablePickup ? 1 : 0
    );

    return true;
}


// ============================================================
// STATE CHANGE
// ============================================================

static void handleNavStateChange(NavState nav)
{
    motor_control_stop();

    lastNavState = nav;
    turnWatchActive = false;

    weight_detection_reset_side_evidence();

    printNavStateEvent(nav);

    if (nav == ROAMING)
    {
        pursuit_reset();
        roaming_reset();
        return;
    }

    if (nav == REVERSING)
    {
        reversing_start();
        return;
    }

    if (nav == HOMING)
    {
        homing_start();
        return;
    }
}


// ============================================================
// TURN WATCHDOG
// ============================================================

static bool checkTurnTimeout(NavState nav)
{
    if (!motor_control_is_turning())
    {
        turnWatchActive = false;
        return false;
    }

    if (!turnWatchActive)
    {
        turnStartedAt = millis();
        turnWatchActive = true;

        return false;
    }

    if (millis() - turnStartedAt <
        NAV_TURN_TIMEOUT_MS)
    {
        return false;
    }

    motor_control_stop();
    turnWatchActive = false;

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",TURN_TIMEOUT,");
    debugNav.println(
        getNavStateName()
    );

    if (nav == PURSUIT)
    {
        setStateFlag(
            &STATE_FLAGS.target_lost
        );
    }
    // else if (nav == ROAMING)
    // {
    // }
    else if (nav == HOMING)
    {
        homing_on_turn_timeout();
    }

    return true;
}


static float headingChange(
    float a,
    float b)
{
    float diff =
        fabsf(a - b);

    if (diff > 180.0f)
    {
        diff =
            360.0f - diff;
    }

    return diff;
}


static void resetMotionWatchdog(
    NavState nav)
{
    lastPhysicalMotionAt =
        millis();

    lastPhysicalLeftMm =
        encoders_get_left_distance_mm();

    lastPhysicalRightMm =
        encoders_get_right_distance_mm();

    lastPhysicalHeading =
        imu_get_heading();

    motionWatchState =
        nav;
}


static bool checkMotionWatchdog(
    NavState nav)
{
    // Only watchdog states in which the chassis
    // should eventually be doing something.
    if (nav != ROAMING &&
        nav != PURSUIT &&
        nav != HOMING)
    {
        resetMotionWatchdog(nav);
        return false;
    }

    // Don't back away after successfully docking at home.
    if (nav == HOMING &&
        homing_is_docking())
    {
        resetMotionWatchdog(nav);
        return false;
    }

    // New navigation state = start a fresh timer.
    if (nav != motionWatchState ||
        lastPhysicalMotionAt == 0)
    {
        resetMotionWatchdog(nav);
        return false;
    }

    float leftMm =
        encoders_get_left_distance_mm();

    float rightMm =
        encoders_get_right_distance_mm();

    float heading =
        imu_get_heading();

    bool encoderMoved =
        fabsf(
            leftMm -
            lastPhysicalLeftMm
        ) >= 25.0f
        ||
        fabsf(
            rightMm -
            lastPhysicalRightMm
        ) >= 25.0f;

    bool headingMoved =
        headingChange(
            heading,
            lastPhysicalHeading
        ) >= 5.0f;

    // Any genuine physical movement means the
    // navigation system is alive.
    if (encoderMoved ||
        headingMoved)
    {
        resetMotionWatchdog(nav);
        return false;
    }

    if (millis() -
            lastPhysicalMotionAt <
        NAV_STUCK_TIMEOUT_MS)
    {
        return false;
    }

    // =================================================
    // NOTHING PHYSICALLY MOVED FOR 20 SECONDS
    // =================================================

    debugNav.print(
        "NAV_EVENT,"
    );

    debugNav.print(
        millis()
    );

    debugNav.print(
        ",STUCK_WATCHDOG,"
    );

    debugNav.println(
        getNavStateName()
    );

    motor_control_stop();

    // If pursuit got stuck, abandon that weight before
    // escaping so we don't immediately resume bad pursuit.
    if (nav == PURSUIT)
    {
        setStateFlag(
            &STATE_FLAGS.target_lost
        );
    }

    reversing_set_reason(
        REVERSE_NORMAL
    );

    setStateFlag(
        &STATE_FLAGS.reverse_triggered
    );

    // Prevent it firing repeatedly while the state machine
    // is transitioning into REVERSING.
    resetMotionWatchdog(nav);

    return true;
}

// ============================================================
// MAIN NAVIGATOR
// ============================================================

void navigator_exe()
{
    if (!navigatorEnabled)
    {
        return;
    }

    NavState nav =
        getNavState();

    if (nav != lastNavState)
    {
        handleNavStateChange(nav);
    }

    if (!imu_is_online() ||
        !isfinite(imu_get_heading()))
    {
        debugNav.print("NAV_EVENT,");
        debugNav.print(millis());
        debugNav.println(",NAVIGATOR_STOP,IMU");

        navigator_stop();

        return;
    }

    if (checkMotionWatchdog(nav))
    {
        return;
    }

    bool suppressPitchEscape =
        nav == HOMING &&
        homing_is_docking();

    if (reversing_check_for_pitch_escape(
            nav,
            suppressPitchEscape))
    {
        return;
    }

    if (checkTurnTimeout(nav))
    {
        return;
    }

    switch (nav)
    {
        case ROAMING:
            roaming_update();
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

    // A controller may have started a turn during this update.
    if (motor_control_is_turning() &&
        !turnWatchActive)
    {
        turnStartedAt = millis();
        turnWatchActive = true;
    }
}


void print_navigator_state()
{
    Serial.print("NAV_STATE,");
    Serial.println(
        getNavStateName()
    );

    Serial2.print("NAV_STATE,");
    Serial2.println(
        getNavStateName()
    );
}
