#include "colour_sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>



static Adafruit_TCS34725 colourSensor(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);


static bool colourSensorOnline = false;

static unsigned long lastPrintTime = 0;

const unsigned long COLOUR_PRINT_PERIOD_MS = 250;



bool colour_sensor_init()
{
    Serial.println(
        "Starting colour sensor setup..."
    );


    // Colour sensor is on second I2C bus
    Wire1.begin();


    colourSensorOnline =
        colourSensor.begin(
            TCS34725_ADDRESS,
            &Wire1
        );


    if (!colourSensorOnline)
    {
        Serial.println(
            "ERROR: TCS34725 colour sensor not detected"
        );

        return false;
    }


    Serial.println(
        "TCS34725 colour sensor detected"
    );


    return true;
}



void colour_sensor_update()
{
    if (!colourSensorOnline)
    {
        return;
    }

    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;


    colourSensor.getRawData(
        &red,
        &green,
        &blue,
        &clear
    );
}