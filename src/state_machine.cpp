
#include "state_machine.h"
// #include <stdexcept>
#include <Arduino.h>


// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)

NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = LOWERING_VERT;
CollectState prev_collect_state = LOWERING_VERT;

// StateFlags STATE_FLAGS;

const int delay_idle = 100;
const int delay_collect_vert = 100;
const int delay_collect_hor = 100;
const int delay_dropping = 100;
const int delay_retrun = 100;
const int MAX_ITERATIONS = 3;

int current_pickup_iterations = 0;

unsigned long timeInNavState();
unsigned long timeInCollectState();

static unsigned long navStateEnteredAt = 0;
static unsigned long collectStateEnteredAt = 0;

const unsigned long TIMER_1_DURATION = 2000;
const unsigned long TIMER_2_DURATION = 2000;
const unsigned long TIMER_3_DURATION = 2000;
const unsigned long TIMER_4_DURATION = 2000;


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

void check_timers() { 
    switch (current_collect_state) { 
        case LOWERING_VERT: 
            if (timeInCollectState() >= TIMER_1_DURATION) { 
                setStateFlag(&STATE_FLAGS.timer_1);
            } 
            break; 

        case LOWERING_HORI: 
            if (timeInCollectState() >= TIMER_2_DURATION) { 
                setStateFlag(&STATE_FLAGS.timer_2); 
            } 
            break; 

        case PICKING_UP: 
            if (timeInCollectState() >= TIMER_3_DURATION) { 
                setStateFlag(&STATE_FLAGS.timer_3); 
            } 
            break; 

        case RETURNING_FAILURE: 
            if (timeInCollectState() >= TIMER_4_DURATION) { 
                setStateFlag(&STATE_FLAGS.timer_4);
            } 
            break; 

        default: 
            break; 
    }
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


void updateStateMachine () {
    // restricts state changes to defined stages. logic engine decides which flags to raise
    // changeStates();

    // checkChangeNavState(REVERSING, &STATE_FLAGS.reverse_triggered);
    // resetStateFlag(&STATE_FLAGS.state_changed);
    check_timers();

    switch(current_nav_state) {
        case STATIONARY:
            checkChangeNavState(ROAMING, &STATE_FLAGS.not_target_weight_onboard);
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
            checkChangeNavState(DROPPING, &STATE_FLAGS.home_reached);
            break;
        
        case DROPPING:
            checkChangeNavState(STATIONARY, &STATE_FLAGS.dropoff_complete);
            break;

        case COLLECTING:
            if (STATE_FLAGS.collection_complete || STATE_FLAGS.collection_failed) {
                checkChangeNavState(STATIONARY, &STATE_FLAGS.collection_complete);
                checkChangeNavState(ROAMING, &STATE_FLAGS.collection_failed);
                break;
            }

            switch(current_collect_state) {
                case LOWERING_VERT:
                    checkChangeCollectState(VERT_REACHED, &STATE_FLAGS.timer_1);
                    break;

                case VERT_REACHED:
                    checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    checkChangeCollectState(LOWERING_HORI, &STATE_FLAGS.no_vertical);
                    break;

                case LOWERING_HORI:
                    checkChangeCollectState(HORI_REACHED, &STATE_FLAGS.timer_2);
                    break;
                
                case HORI_REACHED:
                    checkChangeCollectState(PICKING_UP, &STATE_FLAGS.magnet_hit);
                    checkChangeCollectState(RETURNING_FAILURE, &STATE_FLAGS.no_horisontal);
                    break;

                case PICKING_UP:
                    checkChangeCollectState(RETURNING_SUCCESS, &STATE_FLAGS.timer_3);
                    break;

                case RETURNING_SUCCESS:
                    setStateFlag(&STATE_FLAGS.collection_complete);
                    break;
                
                case RETURNING_FAILURE:
                    checkChangeCollectState(IDLE, &STATE_FLAGS.timer_4);
                    break;
                
                case IDLE:
                    if(current_pickup_iterations >= MAX_ITERATIONS) {
                        setStateFlag(&STATE_FLAGS.collection_failed);
                    } else {
                        checkChangeCollectState(LOWERING_VERT, &STATE_FLAGS.can_iterate); // should be on by default
                    }
            }
            break;
        }

};

