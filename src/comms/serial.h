#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>


enum RobotCommand
{
    CMD_NONE,

    CMD_FORWARD,
    CMD_REVERSE,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,

    CMD_SPEED_UP,
    CMD_SPEED_DOWN,

    CMD_MOTORS_ON,
    CMD_MOTORS_OFF,

    CMD_COLLECT,

    CMD_TOF_PRINT,
    CMD_TOF_STREAM_ON,
    CMD_TOF_STREAM_OFF,
    OPEN_GATE,
    CLOSE_GATE,
    WEIGHT_TEST,
    WEIGHT_STOP
};


void serial_init();

RobotCommand serial_exe();


#endif