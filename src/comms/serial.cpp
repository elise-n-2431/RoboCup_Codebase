#include "serial.h"

#include "flag_control.h"
#include "navigator.h"
#include "driving_controller.h"


static String usbBuffer = "";
static String bluetoothBuffer = "";



const int DEFAULT_DRIVE_POWER = 300;
const int MAX_COMMAND_LENGTH = 40;



static RobotCommand parseSimpleCommand(const String &command)
{
    if (command == "w" || command == "forward")
        return CMD_FORWARD;

    if (command == "s" || command == "reverse")
        return CMD_REVERSE;

    if (command == "a" || command == "left")
        return CMD_LEFT;

    if (command == "d" || command == "right")
        return CMD_RIGHT;

    if (command == "x")
        return CMD_STOP;

    if (command == "+")
        return CMD_SPEED_UP;

    if (command == "-")
        return CMD_SPEED_DOWN;

    if (command == "c" || command == "collect")
        return CMD_COLLECT;

    if (command == "mon")
        return CMD_MOTORS_ON;

    if (command == "moff")
        return CMD_MOTORS_OFF;

    if (command == "tof")
        return CMD_TOF_PRINT;

    if (command == "tofon")
        return CMD_TOF_STREAM_ON;

    if (command == "toff")
        return CMD_TOF_STREAM_OFF;

    if (command == "open")
    {
        return OPEN_GATE;
    }
    if (command == "close")
    {
        return CLOSE_GATE;
    }
    return CMD_NONE;
}

// ============================================================
// CONTROL OWNERSHIP
// ============================================================

static void stopAutomaticControl()
{
    navigator_stop();

    if (motor_control_is_active())
    {
        motor_control_stop();
    }
}



static void printHelp(Stream &port)
{
    port.println("Commands:");

    port.println("  w / s / a / d       manual drive");
    port.println("  x or stop           stop all driving/navigation");
    port.println("  + / -               manual drive power");

    port.println("  turn <deg>          relative IMU turn");
    port.println("  drive [power]       drive holding current heading");
    port.println("  nav <heading>       turn to absolute heading then drive");

    port.println("  kp <value>          set turn Kp");
    port.println("  drivekp <value>     set heading-hold Kp");
    port.println("  gains               print current gains");

    port.println("  flag <name>         raise state-machine test flag");
    port.println("  flags               list test flags");

    port.println("  c / collect         collection command");

    port.println("  tof                 print ToF");
    port.println("  tofon               ToF stream on");
    port.println("  toff                ToF stream off");
}


// ============================================================
// FULL COMMAND PARSER
// ============================================================
static RobotCommand parseLine(
    String command,
    Stream &port
)
{
    command.trim();
    command.toLowerCase();


    if (command.length() == 0)
    {
        return CMD_NONE;
    }


    // ========================================================
    // PARAMETERISED CONTROL COMMANDS
    // ========================================================

    if (command == "stop")
    {
        navigator_stop();
        motor_control_stop();

        port.println("Stopped");

        return CMD_STOP;
    }


    if (command.startsWith("nav "))
    {
        float heading =
            command.substring(4).toFloat();

        navigator_setTestHeading(heading);

        port.print("Navigator heading = ");
        port.println(heading);

        return CMD_NONE;
    }


    if (command.startsWith("turn "))
    {
        float angle =
            command.substring(5).toFloat();

        navigator_stop();

        motor_control_turn_relative(angle);

        port.print("Relative turn = ");
        port.println(angle);

        return CMD_NONE;
    }


    if (command == "drive")
    {
        navigator_stop();

        motor_control_drive_current_heading(300);

        port.println("Driving at power 300");

        return CMD_NONE;
    }


    if (command.startsWith("drive "))
    {
        int power =
            command.substring(6).toInt();

        navigator_stop();

        motor_control_drive_current_heading(power);

        port.print("Driving at power ");
        port.println(power);

        return CMD_NONE;
    }


    if (command.startsWith("kp "))
    {
        float kp =
            command.substring(3).toFloat();

        motor_control_set_kp(kp);

        port.print("Turn KP = ");
        port.println(kp);

        return CMD_NONE;
    }


    if (command.startsWith("drivekp "))
    {
        float kp =
            command.substring(8).toFloat();

        motor_control_set_drive_kp(kp);

        port.print("Drive KP = ");
        port.println(kp);

        return CMD_NONE;
    }


    if (command == "gains")
    {
        port.print("Turn KP = ");
        port.println(
            motor_control_get_kp()
        );

        port.print("Drive KP = ");
        port.println(
            motor_control_get_drive_kp()
        );

        return CMD_NONE;
    }


    // ========================================================
    // FLAGS
    // ========================================================

    if (command.startsWith("flag "))
    {
        String flagName =
            command.substring(5);

        if (setFlagByName(flagName))
        {
            port.print("Flag raised: ");
            port.println(flagName);
        }
        else
        {
            port.print("Unknown flag: ");
            port.println(flagName);
        }

        return CMD_NONE;
    }


    if (command == "flags")
    {
        listFlagNames(port);

        return CMD_NONE;
    }


    // ========================================================
    // SIMPLE ROUTED COMMAND
    // ========================================================

    return parseSimpleCommand(command);
}


// ============================================================
// SERIAL PORT READER
// ============================================================

static RobotCommand readPort(
    Stream &port,
    String &buffer
)
{
    while (port.available() > 0)
    {
        char incoming =
            port.read();


        if (incoming == '\r')
        {
            continue;
        }


        if (incoming == '\n')
        {
            RobotCommand command =
                parseLine(
                    buffer,
                    port
                );

            buffer = "";

            return command;
        }


        buffer += incoming;


        if (buffer.length() > 40)
        {
            buffer = "";

            port.println(
                "Command too long - cleared"
            );
        }
    }


    return CMD_NONE;
}


// ============================================================
// INITIALISE SERIAL
// ============================================================

void serial_init()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);


    delay(500);


    Serial.println(
        "Serial interface ready"
    );

    Serial2.println(
        "Serial interface ready"
    );
}


// ============================================================
// RUN SERIAL
// ============================================================
RobotCommand serial_exe()
{
    RobotCommand command =
        readPort(
            Serial,
            usbBuffer
        );


    if (command != CMD_NONE)
    {
        return command;
    }


    return readPort(
        Serial2,
        bluetoothBuffer
    );
}


// ============================================================
// GET SIMPLE COMMAND
// ============================================================
