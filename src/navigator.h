//
// Created by elise on 11/08/2026.
//

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

void navigator_init();
void navigator_exe();

bool navigator_start(bool enablePickup);
void navigator_stop();

void print_navigator_state();

#endif