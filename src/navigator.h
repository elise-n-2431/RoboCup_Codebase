
//
// Created by elise on 11/08/2026.
//

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

struct NavCommand {
    int leftTrack;
    int rightTrack;  
};

void navigator_init();
void navigator_exe();
NavCommand navigator_getCommand();

#endif