#include "logic_engine.h"
#include "state_machine.h"

int current_weights = 0;
int current_iterations = 0;

void choose_action() {
    if (getNavState() != STATIONARY) {
        return;
    }
    if (current_weights < NUM_WEIGHTS) {
        setStateFlag(&STATE_FLAGS.not_target_weight_onboard);
    } else {
        setStateFlag(&STATE_FLAGS.target_weight_onboard);
    }
}

void increment_weights() {
    current_weights += 1;
}

void eval_decide_state() {
    if (getCollectState() != DECIDING) {
        return;
    }

    if (current_iterations >= MAX_ITERATIONS) {
        setStateFlag(&STATE_FLAGS.cant_iterate);
    } else {
        current_iterations += 1;
        setStateFlag(&STATE_FLAGS.can_iterate);
    }
}

void eval_collect_outcome() {
    if (STATE_FLAGS.collection_complete) {
        increment_weights();
    }
}



void logic_exe() {
    choose_action();            
    eval_decide_state();     
    eval_collect_outcome();
}