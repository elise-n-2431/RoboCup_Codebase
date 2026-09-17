#pragma once
#include <TaskScheduler.h>

void tasks_init(void);
void tasks_exe(void);
void tasks_report(Stream& out);

// Runtime tuning
bool tasks_set_period_ms(const char* name, unsigned long ms);
bool tasks_enable(const char* name, bool on);