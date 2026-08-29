#include "serial.h"

#include "flag_control.h"
#include "driving_controller.h"


static String usbBuffer = "";
static String bluetoothBuffer = "";

const int DEFAULT_DRIVE_POWER = 400;
const int MAX_COMMAND_LENGTH = 40;



static void printHelp(Stream &port)
{
    port.println("Commands:");


    port.println("  turn <deg>          relative IMU turn");
    port.println("  drive [power]       drive holding current heading");

    port.println("  kp <value>          set turn Kp");
    port.println("  drivekp <value>     set heading-hold Kp");
    port.println("  gains               print current gains");

    port.println("  flag <name>         raise state-machine test flag");
    port.println("  flags               list test flags");


}

static RobotCommand parseSimpleCommand(const String &command)
{
    if (command == "open")
    {
        return OPEN_GATE;
    }
    if (command == "close")
    {
        return CLOSE_GATE;
    }
    if (command == "help")
    {
        printHelp(Serial2);
        return CMD_NONE;
    }
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


        port.println("Stopped");

        return CMD_STOP;
    }


    


    if (command.startsWith("turn "))
    {
        float angle =
            command.substring(5).toFloat();



        motor_control_turn_relative(angle);

        port.print("Relative turn = ");
        port.println(angle);

        return CMD_NONE;
    }


    if (command == "drive")
    {

        motor_control_drive_current_heading(DEFAULT_DRIVE_POWER);

        port.print("Driving at power ");
        port.println(DEFAULT_DRIVE_POWER);

        return CMD_NONE;
    }


    if (command.startsWith("drive "))
    {
        int power = command.substring(6).toInt();


        motor_control_drive_current_heading(power);

        port.print("Driving at power ");
        port.println(power);

        return CMD_NONE;
    }


    if (command.startsWith("kp "))
    {
        float kp = command.substring(3).toFloat();

        motor_control_set_kp(kp);

        port.print("Turn KP = ");
        port.println(kp);

        return CMD_NONE;
    }


    if (command.startsWith("drivekp "))
    {
        float kp = command.substring(8).toFloat();

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
    return parseSimpleCommand(command);
}



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


        if (buffer.length() > MAX_COMMAND_LENGTH)
        {
            buffer = "";

            port.println(
                "Command too long - cleared"
            );
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


    Serial.println(
        "Serial interface ready"
    );

    Serial2.println(
        "Serial interface ready"
    );
}



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


