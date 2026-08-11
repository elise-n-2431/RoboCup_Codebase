#include "logic_engine.h"
#include "state_machine.h"

int current_weights = 0;
int current_iterations = 0;

void chooseAction() {
    if (getNavState() != STATIONARY) {
        return;
    }
    if (current_weights < NUM_WEIGHTS) {
        setStateFlag(&STATE_FLAGS.not_target_weight_onboard);
    } else {
        setStateFlag(&STATE_FLAGS.target_weight_onboard);
    }
}

void incrementWeights() {
    current_weights += 1;
}

void evaluateDecidingState() {
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

void evaluateCollectionOutcome() {
    if (STATE_FLAGS.collection_complete) {
        incrementWeights();
    }
}

void logic_exe() {
    chooseAction();            
    evaluateDecidingState();     
    evaluateCollectionOutcome();
}