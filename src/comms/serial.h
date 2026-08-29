#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>


enum RobotCommand
{
    CMD_NONE,
    CMD_STOP,
    OPEN_GATE,
    CLOSE_GATE
};


void serial_init();

RobotCommand serial_exe();


#endif