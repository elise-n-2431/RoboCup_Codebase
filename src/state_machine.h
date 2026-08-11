
//
// Created by elise on 20/07/2026.
//

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum NavState {
    STATIONARY,
    ROAMING,
    PURSUIT,
    SORTING,
    COLLECTING,
    HOMING,
    DROPPING
};

enum CollectState {
    LOWERING_VERT,
    VERT_REACHED,
    LOWERING_HORI,
    HORI_REACHED,
    PICKING_UP,
    RETURNING_SUCCESS, // returning to idle after a success
    RETURNING_FAILURE, // returning to idle after a failure
    IDLE
};

struct StateFlags {
    // state flags trigger a change in state
    bool state_changed = false;

    // nav focused
    bool target_identified = false;
    bool reverse_triggered = false;
    bool reverse_complete = false;
    bool home_reached = false;
    bool collection_complete = false;
    bool collection_failed = false;
    bool dropoff_complete = false;
    bool not_target_weight_onboard = false;
    bool target_weight_onboard = false;
    bool dummy_identified = false;
    bool metal_identified = false;

    // collection focused
    bool weight_in_entrance = false; // detected by proximity
    bool magnet_hit = false;   // detected by limit switch
    bool no_vertical = false; // as transition takes time, don't switch states until either trigger
    bool no_horisontal = false; // keep horisontal weight detection seperate so triggers don't overlap
    bool dropping_complete = false;
    bool returning_complete = false;
    bool can_iterate = false;
    bool cant_iterate = false;

    bool timer_1 = false;
    bool timer_2 = false;
    bool timer_3 = false;
    bool timer_4 = false;

};

void updateStateMachine();

void setStateFlag(bool*);

CollectState getCollectState();

extern StateFlags STATE_FLAGS;

#endif


