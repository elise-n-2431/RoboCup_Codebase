#include "logic_engine.h"
#include "state_machine.h"
#include "priority_targets.h"
#include "pose.h"
#include "debug_print.h"

int current_weights = 0;
int current_iterations = 0;



void increment_weights()
{
    current_weights++;


    debugState.print(
        "WEIGHTS_ONBOARD,"
    );

    debugState.println(
        current_weights
    );


    bool targetRemoved =
        priority_targets_remove_nearest(
            pose_get_x_mm(),
            pose_get_y_mm(),
            700.0f
        );


    debugState.print(
        "PRIORITY_TARGET_REMOVED,"
    );

    debugState.println(
        targetRemoved ? 1 : 0
    );


    priority_targets_print(
        Serial
    );

    priority_targets_print(
        Serial2
    );
}

void reset_weights()
{
    current_weights = 0;
}

void reset_collection_iterations()
{
    current_iterations = 0;
}

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


void logic_exe()
{
    if (STATE_FLAGS.dropoff_complete) {
        reset_weights();
        reset_collection_iterations();
        resetStateFlag(&STATE_FLAGS.dropoff_complete);
    }

    choose_action();

    if (getNavState() != COLLECTING || getCollectState() != DECIDING) return;
    if (STATE_FLAGS.can_iterate || STATE_FLAGS.cant_iterate) return;

    if (current_iterations >= MAX_ITERATIONS) {
        setStateFlag(&STATE_FLAGS.cant_iterate);
    } else {
        current_iterations++;
        setStateFlag(&STATE_FLAGS.can_iterate);
    }
}