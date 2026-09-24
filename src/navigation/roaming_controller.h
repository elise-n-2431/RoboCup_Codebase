#ifndef ROAMING_CONTROLLER_H
#define ROAMING_CONTROLLER_H

void roaming_start(bool pickupEnabled);
void roaming_update();
void roaming_reset();

void roaming_turn_timeout();

#endif