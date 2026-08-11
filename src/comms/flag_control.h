#ifndef FLAG_CONTROL_H
#define FLAG_CONTROL_H

#include <Arduino.h>
#include "state_machine.h"

// Sets a STATE_FLAGS member by its field name (e.g. "magnet_hit").
// Returns true if the name matched a known flag, false otherwise.
bool setFlagByName(const String &name);

// Prints every valid flag name, one per line, to the given port.
// Handy for typing "flags" into a bluetooth terminal to see what you can trigger.
void listFlagNames(Stream &port);

#endif