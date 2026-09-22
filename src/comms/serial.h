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

// Actuator commands require an explicitly enabled bench run.
RobotCommand serial_exe(bool allowControl = false);

void serial_set_bench_mode(bool enabled);

#endif