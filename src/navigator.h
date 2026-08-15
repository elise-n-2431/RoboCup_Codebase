
//
// Created by elise on 11/08/2026.
//

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

enum NavMotion
{
    NAV_STOP,
    NAV_TURN,
    NAV_DRIVE
};

struct NavCommand {
    NavMotion motion;
    float heading;
    int power;
};

void navigator_init();
void navigator_exe();
NavCommand navigator_getCommand();

void navigator_setTestHeading(float heading);
void navigator_stop();

bool navigator_is_active();

#endif