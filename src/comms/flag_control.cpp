#include "flag_control.h"

struct FlagEntry {
    const char* name;
    const char* abbrev;
    bool* flag;
};

// Only flags that originate from a physical sensor. Anything set by
// check_timers() or logic_engine.cpp is deliberately excluded — those
// happen automatically regardless of whether a sensor is connected,
// so raising them manually would just fight the real logic.
static const FlagEntry FLAG_TABLE[] = {
    { "target_identified",   "tgt", &STATE_FLAGS.target_identified },
    { "reverse_triggered",   "rev", &STATE_FLAGS.reverse_triggered },
    { "home_reached",        "home",  &STATE_FLAGS.home_reached },
    { "dummy_identified",    "dum", &STATE_FLAGS.dummy_identified },
    { "metal_identified",    "met", &STATE_FLAGS.metal_identified },

    { "weight_in_entrance",  "in", &STATE_FLAGS.weight_in_entrance },
    { "magnet_hit",          "hit", &STATE_FLAGS.magnet_hit },
    { "no_vertical",         "nov", &STATE_FLAGS.no_vertical },
    { "no_horizontal",       "noh", &STATE_FLAGS.no_horizontal },
};

static const size_t FLAG_TABLE_SIZE = sizeof(FLAG_TABLE) / sizeof(FLAG_TABLE[0]);

bool setFlagByName(const String &nameIn)
{
    String name = nameIn;
    name.trim();
    name.toLowerCase();

    for (size_t i = 0; i < FLAG_TABLE_SIZE; i++)
    {
        if (name.equals(FLAG_TABLE[i].name) || name.equals(FLAG_TABLE[i].abbrev))
        {
            setStateFlag(FLAG_TABLE[i].flag);
            return true;
        }
    }
    return false;
}

void listFlagNames(Stream &port)
{
    for (size_t i = 0; i < FLAG_TABLE_SIZE; i++)
    {
        port.print(FLAG_TABLE[i].abbrev);
        port.print("\t- ");
        port.println(FLAG_TABLE[i].name);
    }
}