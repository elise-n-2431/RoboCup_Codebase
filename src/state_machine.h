
//
// Created by elise on 20/07/2026.
//

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

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
    // state flags trigger a change in state

    bool state_changed = false;
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

void updateStateMachine();

void setStateFlag(bool*);

CollectState getCollectState();

extern StateFlags STATE_FLAGS;

#endif


