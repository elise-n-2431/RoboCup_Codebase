#include "colour_sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "state_machine.h"
#include "debug_print.h"
#include <iostream>

struct Colour {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear; 
};

Colour Home = {0, 0, 0, 0};
Colour Current = {0, 0, 0, 0};

static Adafruit_TCS34725 colourSensor(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);


static bool colourSensorOnline = false;

static unsigned long lastPrintTime = 0;

const unsigned long COLOUR_PRINT_PERIOD_MS = 250;

int consecutive_hits = 0;



bool colour_sensor_init()
{
    Serial.println("Starting colour sensor setup...");

    Wire1.begin();

    colourSensorOnline = colourSensor.begin(
        TCS34725_ADDRESS,
        &Wire1
    );

    if (!colourSensorOnline) {
        Serial.println("ERROR: TCS34725 colour sensor not detected");
        return false;
    }

    Serial.println("TCS34725 colour sensor detected");
    delay(200);

    return colour_sensor_capture_home();
}

bool colour_sensor_capture_home()
{
    if (!colourSensorOnline) return false;

    for (int i = 0; i < 5; i++) {
        colourSensor.getRawData(
            &Current.red,
            &Current.green,
            &Current.blue,
            &Current.clear
        );

        delay(60);
    }

    Home = Current;
    consecutive_hits = 0;
    return true;
}



void colour_sensor_update()
{
    if (!colourSensorOnline)
    {
        return;
    }


    colourSensor.getRawData(
        &Current.red,
        &Current.green,
        &Current.blue,
        &Current.clear
    );

    if (getNavState() == HOMING && !STATE_FLAGS.home_reached) 
    { 
        bool cont = true;

        // ignore red 

        uint16_t* homeValues[] = {
            &Home.green,
            &Home.blue,
            &Home.clear
        };

        uint16_t* currentValues[] = {
            &Current.green,
            &Current.blue,
            &Current.clear
        };

        int buffer[] = {
            3,  // green
            5,  // blue
            10  // clear
        };

        for (int i = 0; i < 3 && cont; i++) 
        {
            if (*currentValues[i] < *homeValues[i] - buffer[i] || *currentValues[i] > *homeValues[i] + buffer[i]) 
            {
                cont = false;
            }
        }

        if (cont) 
        { 
            consecutive_hits ++;
        }
        else {
            consecutive_hits = 0;
        }

        if (consecutive_hits >= 5) {
            consecutive_hits = 0;
            setStateFlag(&STATE_FLAGS.home_reached);
        }
    } 

    if (getNavState() == HOMING)
    {
        static unsigned long lastColourDebug = 0;

        if (millis() - lastColourDebug >= 250)
        {
            lastColourDebug = millis();

            debugColour.print("HOME STORED: G=");
            debugColour.print(Home.green);
            debugColour.print(" B=");
            debugColour.print(Home.blue);
            debugColour.print(" C=");
            debugColour.println(Home.clear);

            debugColour.print("HOME CURRENT: G=");
            debugColour.print(Current.green);
            debugColour.print(" B=");
            debugColour.print(Current.blue);
            debugColour.print(" C=");
            debugColour.println(Current.clear);
        }
    }
}


void print_colour() {
    Serial2.print("RED: ");
    Serial2.print(Home.red);
    Serial2.print(", BLUE: ");
    Serial2.print(Home.blue);
    Serial2.print(", GREEN: ");
    Serial2.print(Home.green);
    Serial2.print(", CLEAR: ");
    Serial2.println(Home.clear);

    Serial2.print("RED: ");
    Serial2.print(Current.red);
    Serial2.print(", BLUE: ");
    Serial2.print(Current.blue);
    Serial2.print(", GREEN: ");
    Serial2.print(Current.green);
    Serial2.print(", CLEAR: ");
    Serial2.println(Current.clear);

}