#include "logic_engine.h"
#include "state_machine.h"
#include "inputs/sensors.h"

int current_weights = 0;


void chooseAction() {
    if (current_weights < NUM_WEIGHTS) {
        setStateFlag(&STATE_FLAGS.collection_incomplete);
    } else {
        setStateFlag(&STATE_FLAGS.collection_complete);
    }
}


void incrementWeights() {
    current_weights += 1;
}
