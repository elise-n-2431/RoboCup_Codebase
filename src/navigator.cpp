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


// ============================================================
// NAVIGATOR STATE
// ============================================================

static bool navigatorEnabled = true;
static bool turnWatchActive = false;

static NavState lastNavState = STATIONARY;

static unsigned long turnStartedAt = 0;

static const unsigned long NAV_TURN_TIMEOUT_MS = 8000;


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
    else if (nav == ROAMING)
    {
        roaming_turn_timeout();
    }
    else if (nav == HOMING)
    {
        homing_start();
    }

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