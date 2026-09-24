#include "serial.h"
#include "debug_print.h"
#include "arena_config.h"
#include "pose.h"
#include "map.h"
#include "flag_control.h"
#include "driving_controller.h"
#include "navigator.h"
#include "state_machine.h"
#include "inputs/tof_expander.h"
#include "outputs/smart_servo.h"
#include "priority_targets.h"
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>

static String usbBuffer, bluetoothBuffer;
static bool usbOverflow = false, bluetoothOverflow = false;
static int manualPower = 280;

static void printConfig(Stream& port)
{
    const ArenaConfig& c = arena_get_config();

    // CONFIG,colour,homeX,homeY,startX,startY,heading,locked,started,pickup
    port.print("CONFIG,");
    port.print(c.colour == HomeColour::BLUE ? "blue" : "green");

    port.print(','); port.print(c.homeX);
    port.print(','); port.print(c.homeY);
    port.print(','); port.print(c.startX);
    port.print(','); port.print(c.startY);
    port.print(','); port.print(c.startHeading);
    port.print(','); port.print(arena_is_locked() ? 1 : 0);
    port.print(','); port.print(arena_run_started() ? 1 : 0);
    port.print(','); port.println(c.pickupEnabled ? 1 : 0);
}

// Reject missing/extra tokens, non-numbers, NaN, infinity and overflow.
static bool parseNumbers(const char* text, float* values, int count)
{
    for (int i = 0; i < count; ++i) {
        while (isspace(static_cast<unsigned char>(*text))) ++text;
        if (!*text) return false;

        char* end;
        errno = 0;
        values[i] = strtof(text, &end);

        if (end == text || errno == ERANGE || !isfinite(values[i]))
            return false;

        if (*end && !isspace(static_cast<unsigned char>(*end)))
            return false;

        text = end;
    }

    while (isspace(static_cast<unsigned char>(*text))) ++text;
    return *text == '\0';
}

static void configResult(Stream& port, bool ok)
{
    if (!ok) {
        port.println(
            arena_is_locked()
                ? "ERR,config,locked"
                : "ERR,config,invalid"
        );
        return;
    }

    port.println("OK,config");
    printConfig(port);
}

static bool handleConfig(const String& command, Stream& port)
{
    if (command == "config") {
        printConfig(port);
    }
    else if (command == "config lock") {
        arena_lock();
        configResult(port, true);
    }
    else if (command == "base blue" || command == "base green") {
        configResult(
            port,
            arena_set_home_colour(
                command == "base blue"
                    ? HomeColour::BLUE
                    : HomeColour::GREEN
            )
        );
    }
    else if (command.startsWith("home ")) {
        bool ok = false;

        if (command == "home sw")
            ok = arena_set_home_corner(HomeCorner::SW);

        if (command == "home se")
            ok = arena_set_home_corner(HomeCorner::SE);

        if (command == "home nw")
            ok = arena_set_home_corner(HomeCorner::NW);

        if (command == "home ne")
            ok = arena_set_home_corner(HomeCorner::NE);

        //if (ok) map_init();
        configResult(port, ok);
    }
    else if (command.startsWith("homepos ")) {
        float v[2];

        bool ok =
            parseNumbers(command.c_str() + 8, v, 2) &&
            arena_set_home_position(v[0], v[1]);

        //if (ok) map_init();
        configResult(port, ok);
    }
    else if (command.startsWith("startpose ")) {
        float v[3];

        bool ok =
            parseNumbers(command.c_str() + 10, v, 3) &&
            arena_set_start_pose(v[0], v[1], v[2]);

        if (ok) {
            pose_reset();
            //map_init();
        }

        configResult(port, ok);
    }
    else {
        return false;
    }

    return true;
}

static bool handleTargets(
    const String& command,
    Stream& port)
{
    // --------------------------------------------------------
    // Read-only target list.
    // Allowed even after configuration has been locked.
    // --------------------------------------------------------

    if (command == "targets")
    {
        priority_targets_print(port);
        return true;
    }


    // From here down, target editing is PRE-RUN ONLY.

    if (arena_is_locked() ||
        arena_run_started())
    {
        if (command == "targets clear" ||
            command.startsWith(
                "target add "))
        {
            port.println(
                "ERR,targets,locked"
            );

            return true;
        }

        return false;
    }


    // --------------------------------------------------------
    // Clear list
    // --------------------------------------------------------

    if (command == "targets clear")
    {
        priority_targets_clear();

        port.println(
            "OK,targets,cleared"
        );

        return true;
    }


    // --------------------------------------------------------
    // Add:
    //
    // target add <x> <y>
    // --------------------------------------------------------

    if (command.startsWith(
            "target add "))
    {
        float values[2];


        if (!parseNumbers(
                command.c_str() + 11,
                values,
                2))
        {
            port.println(
                "ERR,target,invalid"
            );

            return true;
        }


        if (!priority_targets_add(
                values[0],
                values[1]))
        {
            port.println(
                "ERR,target,invalid_or_full"
            );

            return true;
        }


        port.print("OK,target,");
        port.println(
            priority_targets_count() - 1
        );

        return true;
    }


    return false;
}

static bool allowManual(Stream& port)
{
    if (getCollectState() != IDLE ||
        getNavState() == SORTING ||
        getNavState() == COLLECTING) {
        port.println("Manual drive unavailable during collection");
        return false;
    }

    navigator_stop();
    return true;
}

static RobotCommand parseLine(
    String command,
    Stream& port)
{
    command.trim();
    command.toLowerCase();

    if (!command.length()) return CMD_NONE;
    if (handleConfig(command, port))
    {
        return CMD_NONE;
    }

    if (handleTargets(command, port))
    {
        return CMD_NONE;
    }


    if (debug_command(
            command,
            port,
            !arena_run_started())) {
        return CMD_NONE;
    }

    if (command == "help") {
        port.println("base blue|green | home sw|se|nw|ne | homepos <x> <y>");
        port.println("startpose <x> <y> <heading> | config | config lock");
        port.println("Before GO: auto/roam select the startup mode only");
        port.println("debug list|status | debug on|off <module|all>");
        port.println("Read-only: tof | gains | flags | arms | mark fault");
        port.println("Bench mode after GO: auto | roam | stop | open | close");
        port.println("drive [power] | turn <deg> | kp <v> | drivekp <v> | drivepower <v>");
        port.println("left <position> | right <position> | flag <name>");
        port.println("targets | targets clear | target add <x> <y>");
        return CMD_NONE;
    }

    if (command == "tof") {
        tof_print_readings(port);
        return CMD_NONE;
    }

    if (command == "flags") {
        listFlagNames(port);
        return CMD_NONE;
    }

    if (command == "arms") {
        smartservo_print_positions();
        return CMD_NONE;
    }

    if (command == "gains") {
        port.print("Turn KP = ");
        port.println(motor_control_get_kp());

        port.print("Drive KP = ");
        port.println(motor_control_get_drive_kp());

        port.print("Drive Power = ");
        port.println(manualPower);

        return CMD_NONE;
    }

    if (command == "mark fault") {
        port.print("MARK,FAULT,");
        port.println(millis());
        return CMD_NONE;
    }

    if (command == "mode")
    {
        port.println("MODE,COMPETITION");
        return CMD_NONE;
    }

    if ((command == "auto" || command == "roam") &&
        !arena_run_started()) {
        configResult(
            port,
            arena_set_pickup_enabled(command == "auto")
        );
        return CMD_NONE;
    }

    // Never pass a pre-GO actuator/flag command to the router or drivers.
    // In competition firmware these remain disabled after GO as well.
    if (!arena_run_started()) {
        port.println("ERR,control,disabled");
        return CMD_NONE;
    }

    if (command == "stop") return CMD_STOP;
    if (command == "open") return OPEN_GATE;
    if (command == "close") return CLOSE_GATE;

    if (command == "roam" || command == "auto") {
        port.println(
            navigator_start(command == "auto")
                ? "OK,navigator"
                : "ERR,navigator,start"
        );
        return CMD_NONE;
    }

    float value;

    if (command.startsWith("turn ") &&
        parseNumbers(command.c_str() + 5, &value, 1)) {
        if (allowManual(port))
            motor_control_turn_relative(value);
    }
    else if (command == "drive" || command.startsWith("drive ")) {
        if (command != "drive") {
            if (!parseNumbers(command.c_str() + 6, &value, 1)) {
                port.println("ERR,drive,invalid");
                return CMD_NONE;
            }

            manualPower = static_cast<int>(
                constrain(value, 0.0f, 450.0f)
            );
        }

        if (allowManual(port))
            motor_control_drive_current_heading(manualPower);
    }
    else if (command.startsWith("drivepower ") &&
             parseNumbers(command.c_str() + 11, &value, 1)) {
        manualPower = static_cast<int>(
            constrain(value, 0.0f, 450.0f)
        );

        port.print("Drive Power = ");
        port.println(manualPower);
    }
    else if (command.startsWith("drivekp ") &&
             parseNumbers(command.c_str() + 8, &value, 1)) {
        motor_control_set_drive_kp(
            constrain(value, 0.0f, 100.0f)
        );
    }
    else if (command.startsWith("kp ") &&
             parseNumbers(command.c_str() + 3, &value, 1)) {
        motor_control_set_kp(
            constrain(value, 0.0f, 100.0f)
        );
    }
    else if (command.startsWith("left ") ||
             command.startsWith("right ")) {
        const bool left = command.startsWith("left ");

        if (!parseNumbers(
                command.c_str() + (left ? 5 : 6),
                &value,
                1) ||
            value < 0 ||
            value > 1023 ||
            floorf(value) != value) {
            port.println("ERR,arm,invalid");
            return CMD_NONE;
        }

        if (left)
            smartservo_test_left(static_cast<uint16_t>(value));
        else
            smartservo_test_right(static_cast<uint16_t>(value));
    }
    else if (command.startsWith("flag ")) {
        if (!setFlagByName(command.substring(5)))
            port.println("Unknown flag");
    }
    else {
        port.println("ERR,command,unknown_or_invalid");
    }

    return CMD_NONE;
}

static RobotCommand readPort(
    Stream& port,
    String& buffer,
    bool& overflow)
{
    while (port.available())
    {
        char c = port.read();

        if (c == '\r') continue;

        if (c == '\n')
        {
            RobotCommand result =
                overflow
                ? CMD_NONE
                : parseLine(buffer, port);

            buffer = "";
            overflow = false;

            return result;
        }

        if (overflow) continue;

        if (c == '\0' || buffer.length() >= 96)
        {
            buffer = "";
            overflow = true;

            port.println("ERR,command,invalid_or_too_long");
        }
        else
        {
            buffer += c;
        }
    }

    return CMD_NONE;
}

void serial_init()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);

    delay(500);
    Serial.println("Serial interface ready");
}

void serial_exe()
{
    readPort(Serial, usbBuffer, usbOverflow);
    readPort(Serial2, bluetoothBuffer, bluetoothOverflow);
}
