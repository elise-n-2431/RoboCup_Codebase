
#include "state_machine.h"
// #include <stdexcept>
#include "inputs/sensors.h"


// enums and structs moved to h file

// Chose to seperate navigation and collection state machines (may change later)

NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = IDLE;
CollectState prev_collect_state = IDLE;

StateFlags STATE_FLAGS;


const int delay_idle = 100;
const int delay_collect_vert = 100;
const int delay_collect_hor = 100;
const int delay_dropping = 100;
const int delay_retrun = 100;

int current_ticks;
int end_delay_ticks;


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


void checkChangeNavState(NavState navState, bool* flag) {
    if (*flag) {
        prev_nav_state = current_nav_state;
        current_nav_state = navState;
        *flag = false;
        setStateFlag(&STATE_FLAGS.state_changed);
    }

}

// void changeNavState(NavState navState) {
//     prev_nav_state = current_nav_state;
//     current_nav_state = navState;
// }

void checkChangeCollectState(CollectState collectState, bool* flag) {
    if (*flag) {
        if (current_nav_state != STATIONARY) {
            if (collectState != IDLE) {
                // throw std::invalid_argument("Cannot collect weights while moving");
                collectState = IDLE;
            }
        }
        prev_collect_state = current_collect_state;
        current_collect_state = collectState;
        setStateFlag(&STATE_FLAGS.state_changed);
    }
}

// void changeCollectState(CollectState collectState) { 
//     if (current_nav_state != STATIONARY) {
//         if (collectState != IDLE) {
//             throw std::invalid_argument("Cannot collect weights while moving");
//             collectState = IDLE;
//         }
//     }
//     prev_collect_state = current_collect_state;
//     current_collect_state = collectState;
// }

void changeStates() {
    if (STATE_FLAGS.state_changed) {
        switch (current_collect_state) {
            case IDLE:
                end_delay_ticks = current_ticks + delay_idle;
                break;

            case COLLECT_VERTICAL:
                end_delay_ticks = current_ticks + delay_idle;
                break;

            case COLLECT_HORISONTAL:
                end_delay_ticks = current_ticks + delay_idle;
                break;
            
            case DROPPING:
                end_delay_ticks = current_ticks + delay_idle;
                break;

            case RETURNING_TO_IDLE:
                end_delay_ticks = current_ticks + delay_idle;
                break;
            };
            
    } else if (end_delay_ticks <= current_ticks) {
        switch (getCollectState()) {
            case IDLE:
                if (SENSOR_FLAGS.metal_detected) { // currently uses inductive prox
                    setStateFlag(&STATE_FLAGS.weight_in_entrance);
                }
                break;

            case COLLECT_VERTICAL:
                if (SENSOR_FLAGS.limit_switch_on and SENSOR_FLAGS.metal_detected) {
                    setStateFlag(&STATE_FLAGS.weight_detected);
                } else if (!SENSOR_FLAGS.limit_switch_on and SENSOR_FLAGS.metal_detected) {
                    setStateFlag(&STATE_FLAGS.no_vertical);
                } else {
                    // do nothing so far
                }
                break;

            case COLLECT_HORISONTAL:
                if (SENSOR_FLAGS.limit_switch_on and SENSOR_FLAGS.metal_detected) {
                    setStateFlag(&STATE_FLAGS.weight_detected);
                } else if (!SENSOR_FLAGS.limit_switch_on and SENSOR_FLAGS.metal_detected) {
                    setStateFlag(&STATE_FLAGS.no_horisontal);
                } else {
                    // do nothing so far
                }
                break;
            
            case DROPPING:
                setStateFlag(&STATE_FLAGS.dropping_complete);
                break;

            case RETURNING_TO_IDLE:
                setStateFlag(&STATE_FLAGS.returning_complete);
                break;
            }
    }
    }


void updateStateMachine () {
    // restricts state changes to defined stages. logic engine decides which flags to raise
    changeStates();

    checkChangeNavState(REVERSING, &STATE_FLAGS.reverse_triggered);
    resetStateFlag(&STATE_FLAGS.state_changed);

    switch(current_nav_state) {
        case WALL_FINDING:
            checkChangeNavState(TARGET_IDENTIFIED, &STATE_FLAGS.target_identified);
            break;

        case TARGET_IDENTIFIED:
            checkChangeNavState(STATIONARY, &STATE_FLAGS.weight_in_entrance);
            break;

        case REVERSING:
            checkChangeNavState(prev_nav_state, &STATE_FLAGS.reverse_complete);
            break;

        case HOMING:
            checkChangeNavState(STATIONARY, &STATE_FLAGS.home_reached);
            break;

        case STATIONARY:
            checkChangeNavState(HOMING, &STATE_FLAGS.collection_complete);
            checkChangeNavState(WALL_FINDING, &STATE_FLAGS.collection_incomplete);

            switch(current_collect_state) {
                case IDLE:
                    checkChangeCollectState(COLLECT_VERTICAL, &STATE_FLAGS.weight_in_entrance);
                    break;

                case COLLECT_VERTICAL:
                    checkChangeCollectState(DROPPING, &STATE_FLAGS.weight_detected);
                    checkChangeCollectState(COLLECT_HORISONTAL, &STATE_FLAGS.no_vertical);
                    break;

                case COLLECT_HORISONTAL:
                    checkChangeCollectState(DROPPING, &STATE_FLAGS.weight_detected);
                    checkChangeCollectState(RETURNING_TO_IDLE, &STATE_FLAGS.no_horisontal);
                    break;

                case DROPPING:
                    checkChangeCollectState(IDLE, &STATE_FLAGS.dropping_complete);
                    break;

                case RETURNING_TO_IDLE:
                    checkChangeCollectState(IDLE, &STATE_FLAGS.returning_complete);
                    break;

            }
            break;
        }

};

