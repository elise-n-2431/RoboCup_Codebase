#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H
#include <Arduino.h>

// Simple switches, no bitmask. Example in setup(): debugNav.enabled = true;
// Disabled printers do not write to either port.
struct DebugPrinter {
    bool enabled = false;

    template<class T>
    void print(const T& value)
    {
        if (enabled) {
            Serial.print(value);
            Serial2.print(value);
        }
    }

    template<class T>
    void print(const T& value, int format)
    {
        if (enabled) {
            Serial.print(value, format);
            Serial2.print(value, format);
        }
    }

    template<class T>
    void println(const T& value)
    {
        if (enabled) {
            Serial.println(value);
            Serial2.println(value);
        }
    }

    void println()
    {
        if (enabled) {
            Serial.println();
            Serial2.println();
        }
    }
};

extern DebugPrinter debugNav, debugState, debugCollection, debugPose;
extern DebugPrinter debugImu, debugTof, debugUltrasound, debugMap;
extern DebugPrinter debugMotor, debugColour;

// Returns true if this was a debug command, including malformed ones.
bool debug_command(
    const String& command,
    Stream& port,
    bool allowChanges
);

#endif