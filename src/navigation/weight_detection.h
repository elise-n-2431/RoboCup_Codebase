#ifndef WEIGHT_DETECTION_H
#define WEIGHT_DETECTION_H

#include <Arduino.h>

enum WeightTargetSide
{
    TARGET_NONE,
    TARGET_LEFT,
    TARGET_RIGHT,
    TARGET_CENTRE
};

WeightTargetSide weight_detection_update();

void weight_detection_reset();
void weight_detection_reset_side_evidence();
void weight_detection_block_for(unsigned long durationMs);

#endif

