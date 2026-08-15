#include "comms/motor_control_comms.h"

#include <Arduino.h>

#include "driving_controller.h"


static String bluetoothBuffer = "";


// ============================================================
// PROCESS COMPLETE COMMAND
// ============================================================

static void processCommand(String command)
{
    command.trim();

    if (command.length() == 0) {
        return;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (command.equalsIgnoreCase("stop"))
    {
        motor_control_stop();

        Serial.println("Motor control stopped");
        Serial2.println("Motor control stopped");

        return;
    }


    // --------------------------------------------------------
    // PRINT CURRENT GAINS
    // --------------------------------------------------------

    if (command.equalsIgnoreCase("gains"))
    {
        Serial2.print("KP = ");
        Serial2.println(motor_control_get_kp());

        Serial2.print("KD = ");
        Serial2.println(motor_control_get_kd());

        return;
    }


    // --------------------------------------------------------
    // SET KP
    // --------------------------------------------------------

    if (command.startsWith("kp "))
    {
        float kp =
            command.substring(3).toFloat();

        motor_control_set_kp(kp);


        Serial.print("KP changed to: ");
        Serial.println(kp);

        Serial2.print("KP = ");
        Serial2.println(kp);

        return;
    }


    // --------------------------------------------------------
    // SET KD
    // --------------------------------------------------------

    if (command.startsWith("kd "))
    {
        float kd =
            command.substring(3).toFloat();

        motor_control_set_kd(kd);


        Serial.print("KD changed to: ");
        Serial.println(kd);

        Serial2.print("KD = ");
        Serial2.println(kd);

        return;
    }


    // --------------------------------------------------------
    // RELATIVE TURN
    // --------------------------------------------------------

    if (command.startsWith("turn "))
    {
        float angle =
            command.substring(5).toFloat();


        Serial.print("Turn request: ");
        Serial.println(angle);


        Serial2.print("Turning ");
        Serial2.println(angle);


        motor_control_turn_relative(angle);

        return;
    }


    // --------------------------------------------------------
    // UNKNOWN COMMAND
    // --------------------------------------------------------

    Serial2.print("Unknown command: ");
    Serial2.println(command);
}


// ============================================================
// INITIALISE
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


        if (incoming == '\r') {
            continue;
        }


        if (incoming == '\n')
        {
            processCommand(bluetoothBuffer);

            bluetoothBuffer = "";

            continue;
        }


        bluetoothBuffer += incoming;
    }
}