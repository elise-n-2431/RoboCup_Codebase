#include "logic_engine.h"
#include "state_machine.h"

int current_weights = 0;


void chooseAction() {
    if (current_weights < NUM_WEIGHTS) {
        setStateFlag(&STATE_FLAGS.not_target_weight_onboard);
    } else {
        setStateFlag(&STATE_FLAGS.target_weight_onboard);
    }
}


void incrementWeights() {
    current_weights += 1;
}
