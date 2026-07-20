
#include "state_machine.h"
#include <stdexcept>


// Chose to seperate navigation and collection state machines (may change later)

enum NavState {
    WALL_FINDING,
    TARGET_IDENTIFIED,
    REVERSING,
    HOMING,
    STATIONARY
};

enum CollectState {
    IDLE,
    COLLECT_VERTICAL,
    COLLECT_HORISONTAL,
    DROPPING,
    RETURNING_TO_IDLE
};

struct StateFlags {
    // nav focused
    bool target_identified = false;
    bool reverse_triggered = false;
    bool reverse_complete = false;
    bool home_reached = false;
    bool collection_complete = false;
    bool collection_incomplete = false;

    bool dropoff_complete = false;

    // collection focused
    bool weight_in_entrance = false; // detected by proximity
    bool weight_detected = false;   // detected by limit switch
    bool no_vertical = false; // as transition takes time, don't switch states until either trigger
    bool no_horisontal = false; // keep horisontal weight detection seperate so triggers don't overlap
    bool dropping_complete = false;
    bool returning_complete = false;
};


NavState current_nav_state = STATIONARY;
NavState prev_nav_state = STATIONARY;

CollectState current_collect_state = IDLE;
CollectState prev_collect_state = IDLE;

StateFlags state_flags;


void setFlag(bool* flag) {
    *flag = true;
}


void checkChangeNavState(NavState navState, bool* flag) {
    if (*flag) {
        prev_nav_state = current_nav_state;
        current_nav_state = navState;
        *flag = false;
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
                throw std::invalid_argument("Cannot collect weights while moving");
                collectState = IDLE;
            }
        }
        prev_collect_state = current_collect_state;
        current_collect_state = collectState;
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


// main checks for flags raised
void updateStateMachine () {

    checkChangeNavState(REVERSING, &state_flags.reverse_triggered);

    switch(current_nav_state) {
        case WALL_FINDING:
            checkChangeNavState(TARGET_IDENTIFIED, &state_flags.target_identified);
            break;

        case TARGET_IDENTIFIED:
            checkChangeNavState(STATIONARY, &state_flags.weight_in_entrance);
            break;

        case REVERSING:
            checkChangeNavState(prev_nav_state, &state_flags.reverse_complete);
            break;

        case HOMING:
            checkChangeNavState(STATIONARY, &state_flags.home_reached);
            break;

        case STATIONARY:
            checkChangeNavState(HOMING, &state_flags.collection_complete);
            checkChangeNavState(WALL_FINDING, &state_flags.collection_incomplete);

            switch(current_collect_state) {
                case IDLE:
                    checkChangeCollectState(COLLECT_VERTICAL, &state_flags.weight_in_entrance);
                    break;

                case COLLECT_VERTICAL:
                    checkChangeCollectState(DROPPING, &state_flags.weight_detected);
                    checkChangeCollectState(COLLECT_HORISONTAL, &state_flags.no_vertical);
                    break;

                case COLLECT_HORISONTAL:
                    checkChangeCollectState(DROPPING, &state_flags.weight_detected);
                    checkChangeCollectState(RETURNING_TO_IDLE, &state_flags.no_horisontal);
                    break;

                case DROPPING:
                    checkChangeCollectState(IDLE, &state_flags.dropping_complete);
                    break;

                case RETURNING_TO_IDLE:
                    checkChangeCollectState(IDLE, &state_flags.returning_complete);
                    break;

            }
            break;
        }

};
