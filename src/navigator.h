
//
// Created by elise on 11/08/2026.
//

#ifndef NAVIGATOR_H
#define NAVIGATOR_H


void navigator_init();

void navigator_exe();


// Temporary navigation test:
// absolute IMU heading, e.g. navigator_setTestHeading(120)
void navigator_setTestHeading(float heading);


void navigator_stop();

bool navigator_is_active();


#endif