#ifndef REJECTED_WEIGHTS_H
#define REJECTED_WEIGHTS_H

void rejected_weights_reset();

void rejected_weights_add_current();

bool rejected_weights_is_near(
    float x_mm,
    float y_mm,
    float &distance_mm
);

int rejected_weights_count();

#endif