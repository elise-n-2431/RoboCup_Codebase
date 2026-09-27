#include "debug_print.h"
#include <string.h>

DebugPrinter debugNav, debugState, debugCollection, debugPose;
DebugPrinter debugImu, debugTof, debugUltrasound, debugMap;
DebugPrinter debugMotor, debugColour;

struct DebugEntry {
    const char* name;
    DebugPrinter* printer;
};

static DebugEntry modules[] = {
    {"nav", &debugNav},
    {"state", &debugState},
    {"collection", &debugCollection},
    {"pose", &debugPose},
    {"imu", &debugImu},
    {"tof", &debugTof},
    {"ultrasound", &debugUltrasound},
    {"map", &debugMap},
    {"motor", &debugMotor},
    {"colour", &debugColour}
};

bool debug_command(
    const String& command,
    Stream& port,
    bool allowChanges)
{
    if (command != "debug" && !command.startsWith("debug "))
        return false;

    if (command == "debug list" || command == "debug status") {
        for (const auto& entry : modules) {
            port.print("DEBUG,");
            port.print(entry.name);
            port.print(',');
            port.println(entry.printer->enabled ? 1 : 0);
        }

        return true;
    }

    const bool on = command.startsWith("debug on ");
    const bool off = command.startsWith("debug off ");

    if (!on && !off) {
        port.println("ERR,debug,use_list_status_on_off");
        return true;
    }

    if (!allowChanges) {
        port.println("ERR,debug,competition_running");
        return true;
    }

    const char* name = command.c_str() + (on ? 9 : 10);
    bool found = false;

    for (auto& entry : modules) {
        if (strcmp(name, "all") == 0 ||
            strcmp(name, entry.name) == 0) {
            entry.printer->enabled = on;
            found = true;
        }
    }

    port.println(found ? "OK,debug" : "ERR,debug,unknown_module");
    return true;
}

