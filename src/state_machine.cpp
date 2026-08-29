
#include "state_machine.h"
// #include <stdexcept>
#include <Arduino.h>


// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)


StateFlags STATE_FLAGS;


// Chose to separate navigation and collection state machines
NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = IDLE;
CollectState prev_collect_state = IDLE;

static NavState reverseReturnState = ROAMING;

static bool pickup_succeeded = false;

unsigned long timeInNavState();
unsigned long timeInCollectState();

static unsigned long navStateEnteredAt = 0;
static unsigned long collectStateEnteredAt = 0;

const unsigned long VERTICAL_LOWER_TIMEOUT_MS = 2000;  
const unsigned long HORIZONTAL_LOWER_TIMEOUT_MS = 2000; 
const unsigned long PICKUP_TIMEOUT_MS = 2000;           
const unsigned long RETURN_TIMEOUT_MS = 2000;

const unsigned long OPENING_TIMEOUT_MS = 2000;
const unsigned long CLOSING_TIMEOUT_MS = 2000;

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
    if (flag == &STATE_FLAGS.target_identified) return "target_identified";
    if (flag == &STATE_FLAGS.reverse_triggered) return "reverse_triggered";
    if (flag == &STATE_FLAGS.home_reached) return "home_reached";
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

    Serial2.print("FLAG,");
    Serial2.println(name);
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


void checkChangeNavState(NavState navState, bool* flag) {
    if (*flag) {
        prev_nav_state = current_nav_state;
        current_nav_state = navState;
        *flag = false;
        navStateEnteredAt = millis();

        Serial2.print("[NAV] ");
        Serial2.print(navStateName(prev_nav_state));
        Serial2.print(" -> ");
        Serial2.println(navStateName(current_nav_state));
    }
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

        Serial2.print("[COLLECT] ");
        Serial2.print(collectStateName(prev_collect_state));
        Serial2.print(" -> ");
        Serial2.println(collectStateName(current_collect_state));
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
            checkChangeNavState(ROAMING, &STATE_FLAGS.not_target_weight_onboard);
            checkChangeNavState(HOMING, &STATE_FLAGS.target_weight_onboard);
            break;

        case ROAMING:
            checkChangeNavState(PURSUIT, &STATE_FLAGS.target_identified);
            break;

        case PURSUIT:
            checkChangeNavState(SORTING, &STATE_FLAGS.weight_in_entrance);
            break;

        case SORTING:
            if (STATE_FLAGS.dummy_identified)
            {
                reverseReturnState = ROAMING; 
            }
            checkChangeNavState(REVERSING, &STATE_FLAGS.dummy_identified);
            checkChangeNavState(COLLECTING, &STATE_FLAGS.metal_identified);
            break;

        case HOMING:
            checkChangeNavState(OPENING, &STATE_FLAGS.home_reached);
            break;

        case OPENING:
            checkChangeNavState(CLOSING, &STATE_FLAGS.opening_complete);
            break;

        case CLOSING:
            checkChangeNavState(STATIONARY, &STATE_FLAGS.closing_complete);
            break;

        case REVERSING:
            checkChangeNavState(reverseReturnState, &STATE_FLAGS.reverse_complete);
            break;

        case COLLECTING:
            if (STATE_FLAGS.collection_complete || STATE_FLAGS.collection_failed) {
                checkChangeNavState(STATIONARY, &STATE_FLAGS.collection_complete);
                checkChangeNavState(ROAMING, &STATE_FLAGS.collection_failed);
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
                    checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    checkChangeCollectState(LOWERING_HORI, &STATE_FLAGS.no_vertical);
                    break;

                case LOWERING_HORI:
                    checkChangeCollectState(HORI_REACHED, &STATE_FLAGS.horizontal_lower_complete);
                    break;

                case HORI_REACHED:
                    checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    checkChangeCollectState(DECIDING, &STATE_FLAGS.no_horizontal);
                    break;

                case PICKING_UP:
                    if (STATE_FLAGS.pickup_complete) {
                        pickup_succeeded = true;
                    }
                    checkChangeCollectState(RETURNING, &STATE_FLAGS.pickup_complete);
                    break;

                case DECIDING:
                    checkChangeCollectState(LOWERING_VERT, &STATE_FLAGS.can_iterate);
                    if (STATE_FLAGS.cant_iterate) {
                        pickup_succeeded = false;
                    }
                    checkChangeCollectState(RETURNING, &STATE_FLAGS.cant_iterate);
                    break;

                case RETURNING:
                    if (STATE_FLAGS.return_complete) {
                        if (pickup_succeeded) {
                            setStateFlag(&STATE_FLAGS.collection_complete);
                        } else {
                            setStateFlag(&STATE_FLAGS.collection_failed);
                        }
                        resetStateFlag(&STATE_FLAGS.return_complete);
                        current_collect_state = IDLE;
                        collectStateEnteredAt = millis();
                    }
                    break;
            }
            break;
    }
}
