#ifndef REVERSING_CONTROLLER_H
#define REVERSING_CONTROLLER_H

#include "../state_machine.h"

enum ReverseReason
{
    REVERSE_NORMAL,
    REVERSE_RAMP,
    REVERSE_CRITICAL_OBSTACLE
};

void reversing_set_reason(ReverseReason reason);

void reversing_set_flat_pitch_reference(float pitch);

void reversing_start();
void reversing_update();

bool reversing_check_for_pitch_escape(NavState nav, bool suppressCheck);

#endif