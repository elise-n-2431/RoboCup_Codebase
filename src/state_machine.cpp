
#include "state_machine.h"
// #include <stdexcept>
#include <Arduino.h>
#include "logic_engine.h"
#include "driving_controller.h"
#include "outputs/pickup_servo.h"
#include "outputs/smart_servo.h"
#include "debug_print.h"
#include "navigation/rejected_weights.h"

// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)


StateFlags STATE_FLAGS;


// Chose to separate navigation and collection state machines
NavState current_nav_state = ROAMING; // -- make ROAMING for testing
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = IDLE;
CollectState prev_collect_state = IDLE;

static NavState reverseReturnState = ROAMING;

static bool pickup_succeeded = false;

unsigned long timeInNavState();
unsigned long timeInCollectState();

static unsigned long navStateEnteredAt = 0;
static unsigned long collectStateEnteredAt = 0;

const unsigned long VERTICAL_LOWER_TIMEOUT_MS = 900;  
const unsigned long HORIZONTAL_LOWER_TIMEOUT_MS = 500; 
const unsigned long PICKUP_TIMEOUT_MS = 1400;           
const unsigned long RETURN_TIMEOUT_MS = 500;

const unsigned long OPENING_TIMEOUT_MS = 6000;
static const unsigned long DROPOFF_DRIVE_AWAY_START_MS = 3000;
static const unsigned long DROPOFF_GATE_CLOSE_MS = 5000;

static const int DROPOFF_EXIT_POWER = 430;
const unsigned long CLOSING_TIMEOUT_MS = 2000;

static const int DROPOFF_SHAKE_POWER = 320;

static const unsigned long SHAKE_START_MS = 500;
static const unsigned long SHAKE_FORWARD_1_END_MS = 680;
static const unsigned long SHAKE_STOP_1_END_MS = 800;
static const unsigned long SHAKE_REVERSE_END_MS = 980;
static const unsigned long SHAKE_STOP_2_END_MS = 1100;
static const unsigned long SHAKE_FORWARD_2_END_MS = 1280;

static int dropoffShakeStage = -1;

static void updateDropoffShake()
{
    unsigned long t =
        timeInNavState();

    int newStage = 0;

    if (t < 500)
    {
        newStage = 0;
    }
    else if (t < 680)
    {
        newStage = 1;
    }
    else if (t < 800)
    {
        newStage = 2;
    }
    else if (t < 980)
    {
        newStage = 3;
    }
    else if (t < 1100)
    {
        newStage = 4;
    }
    else if (t < 1280)
    {
        newStage = 5;
    }
    else if (t <
             DROPOFF_DRIVE_AWAY_START_MS)
    {
        newStage = 6;
    }
    else if (t <
             DROPOFF_GATE_CLOSE_MS)
    {
        newStage = 7;
    }
    else if (t <
             OPENING_TIMEOUT_MS)
    {
        newStage = 8;
    }
    else
    {
        newStage = 9;
    }

    if (newStage ==
        dropoffShakeStage)
    {
        return;
    }

    dropoffShakeStage =
        newStage;

    switch (dropoffShakeStage)
    {
        case 0:
            motor_control_stop();
            break;

        case 1:
            motor_control_drive_current_heading(
                DROPOFF_SHAKE_POWER
            );

            debugState.println(
                "DROPOFF_SHAKE,FORWARD_1"
            );
            break;

        case 2:
            motor_control_stop();
            break;

        case 3:
            motor_control_reverse(
                DROPOFF_SHAKE_POWER
            );

            debugState.println(
                "DROPOFF_SHAKE,REVERSE"
            );
            break;

        case 4:
            motor_control_stop();
            break;

        case 5:
            motor_control_drive_current_heading(
                DROPOFF_SHAKE_POWER
            );

            debugState.println(
                "DROPOFF_SHAKE,FORWARD_2"
            );
            break;

        case 6:
            motor_control_stop();
            break;

        case 7:
            // Robot has already turned 180 degrees
            // during homing, so forward should take
            // it out of its own base.
            motor_control_drive_current_heading(
                DROPOFF_EXIT_POWER
            );

            debugState.println(
                "DROPOFF_EXIT,DRIVING"
            );
            break;

        case 8:
            // Keep moving while closing the gate.
            gateClose();

            motor_control_drive_current_heading(
                DROPOFF_EXIT_POWER
            );

            debugState.println(
                "DROPOFF_EXIT,GATE_CLOSING"
            );
            break;

        default:
            motor_control_stop();
            break;
    }
}

unsigned long timeInNavState()
{
    return millis() - navStateEnteredAt;
}

unsigned long timeInCollectState()
{
    return millis() - collectStateEnteredAt;
}

//prinout for the gui 
static const char* stateFlagName(bool* flag)
{
    if (flag == &STATE_FLAGS.target_lost) return "target_lost";
    if (flag == &STATE_FLAGS.target_identified) return "target_identified";
    if (flag == &STATE_FLAGS.reverse_triggered) return "reverse_triggered";
    if (flag == &STATE_FLAGS.home_reached) return "home_reached";
    if (flag == &STATE_FLAGS.home_docked) return "home_docked";
    if (flag == &STATE_FLAGS.collection_complete) return "collection_complete";
    if (flag == &STATE_FLAGS.collection_failed) return "collection_failed";
    if (flag == &STATE_FLAGS.dropoff_complete) return "dropoff_complete";
    if (flag == &STATE_FLAGS.not_target_weight_onboard) return "not_target_weight_onboard";
    if (flag == &STATE_FLAGS.target_weight_onboard) return "target_weight_onboard";
    if (flag == &STATE_FLAGS.dummy_identified) return "dummy_identified";
    if (flag == &STATE_FLAGS.metal_identified) return "metal_identified";

    if (flag == &STATE_FLAGS.weight_in_entrance) return "weight_in_entrance";
    if (flag == &STATE_FLAGS.magnet_hit) return "magnet_hit";
    if (flag == &STATE_FLAGS.no_vertical) return "no_vertical";
    if (flag == &STATE_FLAGS.no_horizontal) return "no_horizontal";
    if (flag == &STATE_FLAGS.can_iterate) return "can_iterate";
    if (flag == &STATE_FLAGS.cant_iterate) return "cant_iterate";

    if (flag == &STATE_FLAGS.vertical_lower_complete) return "vertical_lower_complete";
    if (flag == &STATE_FLAGS.horizontal_lower_complete) return "horizontal_lower_complete";
    if (flag == &STATE_FLAGS.pickup_complete) return "pickup_complete";
    if (flag == &STATE_FLAGS.return_complete) return "return_complete";
    if (flag == &STATE_FLAGS.reverse_complete) return "reverse_complete";
    if (flag == &STATE_FLAGS.opening_complete) return "opening_complete";
    if (flag == &STATE_FLAGS.closing_complete) return "closing_complete";

    return "unknown";
}

void setStateFlag(bool* flag)
{
    if (*flag) return;

    *flag = true;

    const char* name = stateFlagName(flag);

    Serial.print("FLAG,");
    Serial.println(name);

    // Serial2.print("FLAG,");
    // Serial2.println(name);
}

void resetStateFlag(bool* flag) {
    if (*flag) {
        *flag = false;
    }
}

CollectState getCollectState() {
    return current_collect_state;
}

NavState getNavState() {
    return current_nav_state;
}

static const char* navStateName(NavState state)
{
    switch (state)
    {
        case STATIONARY: return "STATIONARY";
        case ROAMING:     return "ROAMING";
        case PURSUIT:     return "PURSUIT";
        case SORTING:     return "SORTING";
        case COLLECTING:  return "COLLECTING";
        case HOMING:      return "HOMING";
        case OPENING:     return "OPENING";
        case CLOSING:     return "CLOSING";
        case REVERSING:   return "REVERSING";
        default:          return "UNKNOWN_NAV_STATE";
    }
}

static const char* collectStateName(CollectState state)
{
    switch (state)
    {
        case IDLE:          return "IDLE";
        case LOWERING_VERT: return "LOWERING_VERT";
        case VERT_REACHED:  return "VERT_REACHED";
        case LOWERING_HORI: return "LOWERING_HORI";
        case HORI_REACHED:  return "HORI_REACHED";
        case PICKING_UP:    return "PICKING_UP";
        case RETURNING:     return "RETURNING";
        case DECIDING:      return "DECIDING";
        default:            return "UNKNOWN_COLLECT_STATE";
    }
}

const char* getNavStateName()
{
    return navStateName(current_nav_state);
}

const char* getCollectStateName()
{
    return collectStateName(current_collect_state);
}


void checkChangeNavState(NavState navState, bool* flag)
{
    if (!*flag) return;

    motor_control_stop();
    prev_nav_state = current_nav_state;
    current_nav_state = navState;
    *flag = false;
    navStateEnteredAt = millis();

    bool dropoff = STATE_FLAGS.dropoff_complete;
    STATE_FLAGS.dropoff_complete = dropoff;

    if (navState == COLLECTING) {
        current_collect_state = IDLE;
        pickup_succeeded = false;
        reset_collection_iterations();
    }

    if (navState == OPENING)
    {
        dropoffShakeStage = -1;
        gateOpen();
    }

    debugState.print("[NAV] ");
    debugState.print(navStateName(prev_nav_state));
    debugState.print(" -> ");
    debugState.println(navStateName(current_nav_state));
}

void checkChangeCollectState(CollectState collectState, bool* flag) {
    if (*flag) {
        if (current_nav_state != COLLECTING) {
            if (collectState != IDLE) {
                collectState = IDLE;
            }
        }
        prev_collect_state = current_collect_state;
        current_collect_state = collectState;
        *flag = false;
        collectStateEnteredAt = millis();

        debugState.print("[COLLECT] ");
        debugState.print(collectStateName(prev_collect_state));
        debugState.print(" -> ");
        debugState.println(collectStateName(current_collect_state));
    }
}

void check_timers() { 
    // Handles time delays in between states, raises flags when time has elapsed
    
    switch (current_collect_state) { 
        case LOWERING_VERT: 
            if (timeInCollectState() >= VERTICAL_LOWER_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.vertical_lower_complete);
            } 
            break; 

        case LOWERING_HORI: 
            if (timeInCollectState() >= HORIZONTAL_LOWER_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.horizontal_lower_complete); 
            } 
            break; 

        case PICKING_UP: 
            if (timeInCollectState() >= PICKUP_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.pickup_complete); 
            } 
            break; 

        case RETURNING: 
            if (timeInCollectState() >= RETURN_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.return_complete);
            } 
            break;

        default: 
            break; 
    }
    switch (current_nav_state) { 
        case OPENING:
            if (timeInNavState() >= OPENING_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.opening_complete);
            }
            break;

        case CLOSING:
            if (timeInNavState() >= CLOSING_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.closing_complete);
            }
            break;

        default: 
            break; 
    }
}

void updateStateMachine() {
    // handles all state transitions and conditions

    check_timers();
    if (current_nav_state != REVERSING && STATE_FLAGS.reverse_triggered)
    {
        reverseReturnState = current_nav_state;
        checkChangeNavState(REVERSING, &STATE_FLAGS.reverse_triggered);
    }
    
    switch (current_nav_state) {
        case STATIONARY:
            if (STATE_FLAGS.target_weight_onboard) {
                checkChangeNavState(HOMING, &STATE_FLAGS.target_weight_onboard);
            } else checkChangeNavState(ROAMING, &STATE_FLAGS.not_target_weight_onboard);
            break;


        case ROAMING:
            checkChangeNavState(PURSUIT, &STATE_FLAGS.target_identified);
            break;

        case PURSUIT:
            if (STATE_FLAGS.target_lost) {
                checkChangeNavState(ROAMING, &STATE_FLAGS.target_lost);
            } else checkChangeNavState(SORTING, &STATE_FLAGS.weight_in_entrance);
            break;

        case SORTING:
            if (STATE_FLAGS.dummy_identified)
            {
                rejected_weights_add_current();

                smartservo_arms_open();

                reverseReturnState =
                    ROAMING;

                checkChangeNavState(
                    REVERSING,
                    &STATE_FLAGS.dummy_identified
                );
            } else checkChangeNavState(COLLECTING, &STATE_FLAGS.metal_identified);
            break;

        case HOMING:
            checkChangeNavState(OPENING, &STATE_FLAGS.home_docked);
            break;

        case OPENING:
        {
            updateDropoffShake();

            if (STATE_FLAGS.opening_complete)
            {
                checkChangeNavState(
                    STATIONARY,
                    &STATE_FLAGS.opening_complete
                );

                setStateFlag(
                    &STATE_FLAGS.dropoff_complete
                );
            }

            break;
        }
        case CLOSING:
            if (STATE_FLAGS.closing_complete)
            {
                checkChangeNavState(STATIONARY, &STATE_FLAGS.closing_complete);
                setStateFlag(&STATE_FLAGS.dropoff_complete);
            }
            break;

        case REVERSING:
            checkChangeNavState(reverseReturnState, &STATE_FLAGS.reverse_complete);
            break;

        case COLLECTING:
            if (STATE_FLAGS.collection_complete) {
                checkChangeNavState(STATIONARY, &STATE_FLAGS.collection_complete);
                break;
            }

            if (STATE_FLAGS.collection_failed) {
                reverseReturnState = ROAMING;
                checkChangeNavState(REVERSING, &STATE_FLAGS.collection_failed);
                break;
            }


            switch (current_collect_state) {
                case IDLE: {
                    bool start_collecting = true;
                    checkChangeCollectState(LOWERING_VERT, &start_collecting);
                    break;
                }

                case LOWERING_VERT:
                    checkChangeCollectState(VERT_REACHED, &STATE_FLAGS.vertical_lower_complete);
                    break;

                case VERT_REACHED:
                    if (STATE_FLAGS.magnet_hit) {
                        checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    } else checkChangeCollectState(LOWERING_HORI, &STATE_FLAGS.no_vertical);
                    break;

                case LOWERING_HORI:
                    checkChangeCollectState(HORI_REACHED, &STATE_FLAGS.horizontal_lower_complete);
                    break;

                case HORI_REACHED:
                    if (STATE_FLAGS.magnet_hit) {
                        checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    } else checkChangeCollectState(DECIDING, &STATE_FLAGS.no_horizontal);
                    break;

                case PICKING_UP:
                    if (STATE_FLAGS.pickup_complete) {
                        pickup_succeeded = true;
                        checkChangeCollectState(RETURNING, &STATE_FLAGS.pickup_complete);
                    }
                    break;

                case DECIDING:
                    if (STATE_FLAGS.cant_iterate) {
                        pickup_succeeded = false;
                        checkChangeCollectState(RETURNING, &STATE_FLAGS.cant_iterate);
                    } else checkChangeCollectState(LOWERING_VERT, &STATE_FLAGS.can_iterate);
                    break;

                case RETURNING:
                    if (STATE_FLAGS.return_complete) {
                        checkChangeCollectState(IDLE, &STATE_FLAGS.return_complete);

                        if (pickup_succeeded) {
                            increment_weights();
                            smartservo_arms_open();

                            setStateFlag(&STATE_FLAGS.collection_complete);
                        } else {
                            setStateFlag(&STATE_FLAGS.collection_failed);
                        }
                    }
                    break;
            }
            break;
    }
}

void print_state() {
    Serial.println(collectStateName(current_collect_state));
    Serial.println(navStateName(current_nav_state));
}