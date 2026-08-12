#include "serial.h"

static String usbBuffer = "";
static String bluetoothBuffer = "";


// ------------------------------------------------------------
// Convert a single-character command into a RobotCommand
// ------------------------------------------------------------

static RobotCommand parseChar(char c)
{
    switch (c)
    {
        case 'w':
        case 'W':
            return CMD_FORWARD;

        case 's':
        case 'S':
            return CMD_REVERSE;

        case 'a':
        case 'A':
            return CMD_LEFT;

        case 'd':
        case 'D':
            return CMD_RIGHT;

        case 'x':
        case 'X':
        case ' ':
            return CMD_STOP;

        case '+':
            return CMD_SPEED_UP;

        case '-':
            return CMD_SPEED_DOWN;

        case 'c':
        case 'C':
            return CMD_COLLECT;

        default:
            return CMD_NONE;
    }
}


// ------------------------------------------------------------
// Convert a full text command into a RobotCommand
// ------------------------------------------------------------

#include "flag_control.h"   // add at top

static RobotCommand parseLine(String command)
{
    command.trim();
    command.toLowerCase();

    if (command == "mon")  { return CMD_MOTORS_ON; }
    if (command == "moff") { return CMD_MOTORS_OFF; }
    if (command == "tof")  { return CMD_TOF_PRINT; }
    if (command == "tofon"){ return CMD_TOF_STREAM_ON; }
    if (command == "toff") { return CMD_TOF_STREAM_OFF; }

    // "flag <name>" — manually raise a state machine flag for a
    // sensor that's disconnected or not yet firing.
    if (command.startsWith("flag "))
    {
        String flagName = command.substring(5);
        if (!setFlagByName(flagName))
        {
            Serial.print("Unknown flag: ");
            Serial.println(flagName);
        }
        return CMD_NONE;  // handled here, nothing for the main loop to do
    }

    if (command == "flags")
    {
        listFlagNames(Serial);
        return CMD_NONE;
    }

    return CMD_NONE;
}

// ------------------------------------------------------------
// Read commands from either USB Serial or Bluetooth Serial
// ------------------------------------------------------------

static RobotCommand readPort(Stream &port, String &buffer)
{
    while (port.available() > 0)
    {
        char c = port.read();

        // Ignore carriage returns
        if (c == '\r')
        {
            continue;
        }

        // Full text command has arrived
        if (c == '\n')
        {
            if (buffer.length() > 0)
            {
                RobotCommand command = parseLine(buffer);

                buffer = "";

                if (command != CMD_NONE)
                {
                    return command;
                }
            }

            continue;
        }

        // Check immediate single-character commands
        RobotCommand command = parseChar(c);

        if (command != CMD_NONE)
        {
            buffer = "";
            return command;
        }

        // Otherwise build a text command such as "mon"
        buffer += c;

        // Prevent junk filling memory
        if (buffer.length() > 20)
        {
            buffer = "";
        }
    }

    return CMD_NONE;
}

// ------------------------------------------------------------
// Initialise communications
// ------------------------------------------------------------

void serial_init()
{
    // USB serial monitor
    Serial.begin(115200);

    // Bluetooth module
    Serial2.begin(115200);

    delay(500);

    Serial.println("Serial interface ready");
    Serial2.println("Serial interface ready");
}


// ------------------------------------------------------------
// Check both command sources
// ------------------------------------------------------------

RobotCommand serial_get_command()
{
    RobotCommand command;

    // First check USB
    command = readPort(Serial, usbBuffer);

    if (command != CMD_NONE)
    {
        return command;
    }

    // Then check Bluetooth
    command = readPort(Serial2, bluetoothBuffer);

    return command;
}