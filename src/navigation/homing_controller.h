#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

void homing_start();
void homing_update();
void homing_on_turn_timeout();

// Used by ramp detection so the small home-base rim
// does not trigger a reverse during final docking.
bool homing_is_docking();

#endif
