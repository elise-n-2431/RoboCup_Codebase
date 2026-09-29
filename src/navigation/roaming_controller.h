#ifndef ROAMING_CONTROLLER_H
#define ROAMING_CONTROLLER_H

void roaming_start(bool pickupEnabled);
void roaming_update();
void roaming_reset();
bool roaming_check_for_weight(
    bool centreOnly = false
);

#endif
