
#include "state_machine.h"
// #include <stdexcept>
#include <Arduino.h>


// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)

NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = IDLE;
CollectState prev_collect_state = IDLE;

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
const unsigned long REVERSING_TIMEOUT_MS = 2000;


unsigned long timeInNavState()
{
    return millis() - navStateEnteredAt;
}

unsigned long timeInCollectState()
{
    return millis() - collectStateEnteredAt;
}

void setStateFlag(bool* flag) {
    *flag = true;
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


void checkChangeNavState(NavState navState, bool* flag) {
    if (*flag) {
        prev_nav_state = current_nav_state;
        current_nav_state = navState;
        *flag = false;
        navStateEnteredAt = millis();
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
        case REVERSING:
            if (timeInNavState() >= REVERSING_TIMEOUT_MS) { 
                setStateFlag(&STATE_FLAGS.reverse_complete);
            }
            break;

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
    }
}

void updateStateMachine() {
    // handles all state transitions and conditions

    check_timers();
    if (current_nav_state != REVERSING) {
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
            checkChangeNavState(ROAMING, &STATE_FLAGS.dummy_identified);
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
            checkChangeNavState(prev_nav_state, &STATE_FLAGS.reverse_complete);
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
