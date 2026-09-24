#ifndef PURSUIT_CONTROLLER_H
#define PURSUIT_CONTROLLER_H

#include "weight_detection.h"

void pursuit_start(WeightTargetSide target);
void pursuit_update();
void pursuit_reset();

#endif