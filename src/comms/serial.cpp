#include "serial.h"
#include "flag_control.h"
#include "driving_controller.h"
#include "navigator.h"
#include "state_machine.h"
#include "inputs/tof_expander.h"

static String usbBuffer, bluetoothBuffer;
static bool usbOverflow = false, bluetoothOverflow = false;
static int manualPower = 280;

static bool allowManual(Stream &port)
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

// static RobotCommand temp_parseline_override(String command) {
//     if (command == "roam" || command == "auto") {
//         navigator_start(command == "auto");
//         return CMD_NONE;
//     }
// }


static RobotCommand parseLine(String command, Stream &port)
{
    command.trim();
    command.toLowerCase();

    if (!command.length()) return CMD_NONE;
    if (command == "stop") return CMD_STOP;
    if (command == "open") return OPEN_GATE;
    if (command == "close") return CLOSE_GATE;

    if (command == "roam" || command == "auto") {
        navigator_start(command == "auto");
        return CMD_NONE;
    }

    if (command == "help") {
        port.println("roam | auto | stop | drive [power] | turn <deg>");
        port.println("kp <value> | drivekp <value> | drivepower <value> | gains");
        port.println("tof | flag <name> | flags | open | close");
    } else if (command == "tof") {
        tof_print_readings(port);
    } else if (command.startsWith("turn ")) {
        if (allowManual(port)) motor_control_turn_relative(command.substring(5).toFloat());
    } else if (command == "drive" || command.startsWith("drive ")) {
        if (command != "drive") manualPower = constrain(command.substring(6).toInt(), 0, 450);
        if (allowManual(port)) motor_control_drive_current_heading(manualPower);
    } else if (command.startsWith("drivepower ")) {
        manualPower = constrain(command.substring(11).toInt(), 0, 450);
        port.print("Drive Power = ");
        port.println(manualPower);
    } else if (command.startsWith("drivekp ")) {
        motor_control_set_drive_kp(constrain(command.substring(8).toFloat(), 0.0f, 100.0f));
    } else if (command.startsWith("kp ")) {
        motor_control_set_kp(constrain(command.substring(3).toFloat(), 0.0f, 100.0f));
    } else if (command == "gains") {
        port.print("Turn KP = ");
        port.println(motor_control_get_kp());
        port.print("Drive KP = ");
        port.println(motor_control_get_drive_kp());
        port.print("Drive Power = ");
        port.println(manualPower);
    } else if (command == "flags") {
        listFlagNames(port);
    } else if (command.startsWith("flag ")) {
        if (!setFlagByName(command.substring(5))) port.println("Unknown flag");
    } else port.println("Unknown command");

    return CMD_NONE;
}

static RobotCommand readPort(Stream &port, String &buffer, bool &overflow)
{
    while (port.available()) {
        char c = port.read();
        if (c == '\r') continue;

        if (c == '\n') {
            RobotCommand result = overflow ? CMD_NONE : parseLine(buffer, port);
            buffer = "";
            overflow = false;
            return result;
        }

        if (overflow) continue;
        buffer += c;

        if (buffer.length() > 40) {
            buffer = "";
            overflow = true;
            port.println("Command too long");
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
    // Serial2.println("Serial interface ready");
}

RobotCommand serial_exe()
{
    RobotCommand command = readPort(Serial, usbBuffer, usbOverflow);
    if (command != CMD_NONE) return command;

    return readPort(Serial2, bluetoothBuffer, bluetoothOverflow);
}