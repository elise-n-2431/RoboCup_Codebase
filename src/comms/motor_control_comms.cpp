#include "comms/motor_control_comms.h"

#include <Arduino.h>

#include "driving_controller.h"
#include "navigator.h"


static String bluetoothBuffer = "";


// ============================================================
// PROCESS COMMAND
// ============================================================

static void processCommand(String command)
{
    command.trim();
    command.toLowerCase();


    if (command.length() == 0)
    {
        return;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (command == "stop")
    {
        motor_control_stop();

        Serial2.println("Stopped");

        return;
    }

    if (command.startsWith("nav "))
    {
        float heading =
            command.substring(4).toFloat();


        // Cancel any previous direct command
        motor_control_stop();


        // Navigator now owns control
        navigator_setTestHeading(
            heading
        );


        Serial2.print("Navigator heading = ");
        Serial2.println(heading);

        return;
    }


    // --------------------------------------------------------
    // PRINT GAINS
    // --------------------------------------------------------

    if (command == "gains")
    {
        Serial2.print("Turn KP = ");
        Serial2.println(
            motor_control_get_kp()
        );

        Serial2.print("Drive KP = ");
        Serial2.println(
            motor_control_get_drive_kp()
        );

        return;
    }


    // --------------------------------------------------------
    // TURN KP
    // --------------------------------------------------------

    if (command.startsWith("kp "))
    {
        float kp =
            command.substring(3).toFloat();


        motor_control_set_kp(kp);


        Serial2.print("Turn KP = ");
        Serial2.println(kp);

        return;
    }


    // --------------------------------------------------------
    // DRIVE KP
    // --------------------------------------------------------

    if (command.startsWith("drivekp "))
    {
        float kp =
            command.substring(8).toFloat();


        motor_control_set_drive_kp(kp);


        Serial2.print("Drive KP = ");
        Serial2.println(kp);

        return;
    }


    // --------------------------------------------------------
    // RELATIVE TURN
    // --------------------------------------------------------

    if (command.startsWith("turn "))
    {
        float angle =
            command.substring(5).toFloat();


        // Direct control takes ownership
        navigator_stop();


        motor_control_turn_relative(angle);


        Serial2.print("Relative turn ");
        Serial2.println(angle);

        return;
    }


    // --------------------------------------------------------
    // DRIVE AT SPECIFIED POWER
    // --------------------------------------------------------

    if (command.startsWith("drive "))
    {
        int power =
            command.substring(6).toInt();


        // Direct control takes ownership
        navigator_stop();


        motor_control_drive_current_heading(
            power
        );


        Serial2.print("Driving at power ");
        Serial2.println(power);

        return;
    }


    // --------------------------------------------------------
    // DRIVE AT DEFAULT POWER
    // --------------------------------------------------------

    if (command == "drive")
    {
        navigator_stop();

        motor_control_drive_current_heading(300);

        Serial2.println("Driving at power 300");

        return;
    }


    Serial2.print("Unknown command: ");
    Serial2.println(command);
}


// ============================================================
// INIT
// ============================================================

void motor_control_comms_init()
{
    bluetoothBuffer = "";
}


// ============================================================
// UPDATE
// ============================================================

void motor_control_comms_exe()
{
    while (Serial2.available())
    {
        char incoming =
            Serial2.read();


        if (incoming == '\r')
        {
            continue;
        }


        if (incoming == '\n')
        {
            processCommand(
                bluetoothBuffer
            );

            bluetoothBuffer = "";

            continue;
        }


        bluetoothBuffer += incoming;
    }
}