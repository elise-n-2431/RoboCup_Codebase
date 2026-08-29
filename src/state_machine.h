//
// Created by elise on 21/07/2026.
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
    OPENING,
    CLOSING,
    REVERSING   
};

enum CollectState {
    IDLE,      
    LOWERING_VERT,
    VERT_REACHED,
    LOWERING_HORI,
    HORI_REACHED,
    PICKING_UP,
    RETURNING,  
    DECIDING    
};


struct StateFlags {
    // nav focused
    bool target_identified = false;
    bool reverse_triggered = false;
    bool home_reached = false;
    bool collection_complete = false;
    bool collection_failed = false;
    bool dropoff_complete = false;
    bool not_target_weight_onboard = false;
    bool target_weight_onboard = false;
    bool dummy_identified = false;
    bool metal_identified = false;

    // collection focused
    bool weight_in_entrance = false;
    bool magnet_hit = false;
    bool no_vertical = false;
    bool no_horizontal = false;  
    bool can_iterate = false;
    bool cant_iterate = false;

    // timers
    bool vertical_lower_complete = false;
    bool horizontal_lower_complete = false;
    bool pickup_complete = false;
    bool return_complete = false;  
    
    bool reverse_complete = false;
    bool opening_complete = false;
    bool closing_complete = false; 
};

void updateStateMachine();

void setStateFlag(bool*);
void resetStateFlag(bool*);

CollectState getCollectState();
NavState getNavState();

const char* getNavStateName();
const char* getCollectStateName();

extern StateFlags STATE_FLAGS;

#endif