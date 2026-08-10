#include "nav_tof.h"

#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;
const uint8_t FIRST_TOF_ADDRESS = 0x35;
const uint8_t SENSOR_COUNT = 4;

// Purple VL53L1X connections from the working Arduino example.
const uint8_t xshutPins[SENSOR_COUNT] = {4, 0, 1, 2};

SX1509 io;
VL53L0X sensors[SENSOR_COUNT];

uint16_t navTofDistanceMm[SENSOR_COUNT] = {0, 0, 0, 0};
bool navTofSensorOnline[SENSOR_COUNT] = {false, false, false, false};

uint32_t lastNavTofReadMs = 0;


bool nav_tof_init()
{
    Wire.begin();
    Wire.setClock(400000);

    if (!io.begin(SX1509_ADDRESS))
    {
        Serial.println("Failed to find SX1509 at 0x3F");
        Serial2.println("Failed to find SX1509 at 0x3F");
        return false;
    }

    // Shut down all four purple sensors first.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        io.pinMode(xshutPins[i], OUTPUT);
        io.digitalWrite(xshutPins[i], LOW);
    }

    delay(10);

    bool allSensorsStarted = true;

    // Wake, initialise and readdress each sensor individually.
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        // Wake one sensor while the remaining sensors stay shut down.
        io.digitalWrite(xshutPins[i], HIGH);
        delay(100);

        sensors[i].setTimeout(500);

        if (!sensors[i].init())
        {
            Serial.print("Failed to initialise VL53L0X ");
            Serial.println(i);

            Serial2.print("Failed to initialise VL53L0X ");
            Serial2.println(i);

            // Shut it down so it cannot collide with the next sensor.
            io.digitalWrite(xshutPins[i], LOW);
            navTofSensorOnline[i] = false;
            allSensorsStarted = false;
            continue;
        }

        sensors[i].setAddress(FIRST_TOF_ADDRESS + i);

        // Closely follows the VL53L0X ContinuousMultipleSensors example.
        sensors[i].setMeasurementTimingBudget(50000);
        sensors[i].startContinuous(50);

        navTofSensorOnline[i] = true;

        Serial.print("VL53L0X ");
        Serial.print(i);
        Serial.print(" ready at 0x");
        Serial.println(FIRST_TOF_ADDRESS + i, HEX);
    }

    lastNavTofReadMs = millis();
    return allSensorsStarted;
}


void nav_tof_update()
{
    if (millis() - lastNavTofReadMs < 50)
    {
        return;
    }

    lastNavTofReadMs = millis();

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (!navTofSensorOnline[i])
        {
            continue;
        }

        navTofDistanceMm[i] =
        sensors[i].readRangeContinuousMillimeters();

        if (sensors[i].timeoutOccurred())
        {
            navTofSensorOnline[i] = false;
        }
    }
}


void nav_tof_print(Stream &output)
{
    output.print("Front ToF mm: ");

    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        output.print("Sensor ");
        output.print(i);
        output.print("=");

        if (navTofSensorOnline[i])
        {
            output.print(navTofDistanceMm[i]);
        }
        else
        {
            output.print("OFFLINE");
        }

        if (i < SENSOR_COUNT - 1)
        {
            output.print(" | ");
        }
    }

    output.println();
}