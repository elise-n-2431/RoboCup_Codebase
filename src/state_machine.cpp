
#include "state_machine.h"
// #include <stdexcept>
#include "inputs/sensors.h"
#include <Arduino.h>


// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)

NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = LOWERING_VERT;
CollectState prev_collect_state = LOWERING_VERT;

StateFlags STATE_FLAGS;


const int delay_idle = 100;
const int delay_collect_vert = 100;
const int delay_collect_hor = 100;
const int delay_dropping = 100;
const int delay_retrun = 100;

unsigned long timeInNavState();
unsigned long timeInCollectState();

static unsigned long navStateEnteredAt = 0;
static unsigned long collectStateEnteredAt = 0;

// Timer 1 duration
const unsigned long TIMER_1_DURATION = 5000; // 5 seconds


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

bool timer1Expired()
{
    return timeInCollectState() >= TIMER_1_DURATION;
}


void checkChangeNavState(NavState navState, bool flag) {
    if (flag) {
        prev_nav_state = current_nav_state;
        current_nav_state = navState;
        flag = false;
        navStateEnteredAt = millis();
    }

}

void checkChangeCollectState(CollectState collectState, bool flag) {
    if (flag) {
        if (current_nav_state != COLLECTING) {
            if (collectState != IDLE) {
                collectState = IDLE;
            }
        }
        prev_collect_state = current_collect_state;
        current_collect_state = collectState;
        flag = false;
        collectStateEnteredAt = millis();
    }
}




void updateStateMachine () {
    // restricts state changes to defined stages. logic engine decides which flags to raise
    // changeStates();

    // checkChangeNavState(REVERSING, STATE_FLAGS.reverse_triggered);
    // resetStateFlag(STATE_FLAGS.state_changed);

    switch(current_nav_state) {
        case STATIONARY:
            checkChangeNavState(ROAMING, STATE_FLAGS.not_target_weight_onboard);
            break;

        case ROAMING:
            checkChangeNavState(PURSUIT, STATE_FLAGS.target_identified);
            break;

        case PURSUIT:
            checkChangeNavState(SORTING, STATE_FLAGS.weight_in_entrance);
            break;

        case SORTING:
            checkChangeNavState(ROAMING, STATE_FLAGS.dummy_identified);
            checkChangeNavState(COLLECTING, STATE_FLAGS.metal_identified);
            break;

        case HOMING:
            checkChangeNavState(DROPPING, STATE_FLAGS.home_reached);
            break;
        
        case DROPPING:
            checkChangeNavState(STATIONARY, STATE_FLAGS.dropoff_complete);
            break;

        case COLLECTING:
            checkChangeNavState(STATIONARY, STATE_FLAGS.collection_complete);
            checkChangeNavState(ROAMING, STATE_FLAGS.collection_failed);

            switch(current_collect_state) {
                case LOWERING_VERT:
                    checkChangeCollectState(VERT_REACHED, timer1Expired());
                    break;

                case VERT_REACHED:
                    checkChangeCollectState(PICKING_UP, STATE_FLAGS.weight_detected);
                    checkChangeCollectState(LOWERING_HORI, STATE_FLAGS.no_vertical);
                    break;

                case LOWERING_HORI:
                    checkChangeCollectState(HORI_REACHED, "timer 2 elapsed");
                    break;
                
                case HORI_REACHED:
                    checkChangeCollectState(PICKING_UP, STATE_FLAGS.weight_detected);
                    checkChangeCollectState(RETURNING_FAILURE, STATE_FLAGS.no_horisontal);
                    break;

                case PICKING_UP:
                    checkChangeCollectState(RETURNING_SUCCESS, "timer 3 elapsed");
                    break;

                case RETURNING_SUCCESS:
                    // raise pickup_success flag
                    break;
                
                case RETURNING_FAILURE:
                    checkChangeCollectState(IDLE, "timer 4 elapsed");
                    break;
                
                case IDLE:
                    // raise pickup_success flag if iteration limit reached (cant_iterate)
                    checkChangeCollectState(LOWERING_VERT, STATE_FLAGS.can_iterate); // should be on by default

            }
            break;
        }

};

