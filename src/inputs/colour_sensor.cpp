#include "colour_sensor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "state_machine.h"
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

    colour_sensor_update();

    Home = Current;

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