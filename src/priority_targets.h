#ifndef PRIORITY_TARGETS_H
#define PRIORITY_TARGETS_H

#include <Arduino.h>


static const uint8_t MAX_PRIORITY_TARGETS = 12;


struct PriorityTarget
{
    float x;
    float y;
};

bool priority_targets_remove_nearest(
    float x,
    float y,
    float maxDistanceMm
);


void priority_targets_clear();

bool priority_targets_add(
    float x,
    float y
);

bool priority_targets_remove(
    uint8_t index
);

uint8_t priority_targets_count();

bool priority_targets_get(
    uint8_t index,
    float &x,
    float &y
);

void priority_targets_print(
    Stream &port
);

void priority_targets_load_starting_weights();

#endif