#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

#include "tof_expander.h"


static SX1509 tofExpander;

const int TOF_EXPANDER_ADDRESS = 0x71;




// purple sensors
static VL53L0X tof0;
static VL53L0X tof5;
static VL53L0X tof6;
static VL53L0X tof7;

// black sensors
static VL53L1X tof1;
static VL53L1X tof2;
static VL53L1X tof3;
static VL53L1X tof4;



//found from testing

const int TOF0_XSHUT = 0;

const int TOF1_XSHUT = 3;
const int TOF2_XSHUT = 4;
const int TOF3_XSHUT = 5;
const int TOF4_XSHUT = 6;

const int TOF5_XSHUT = 7;
const int TOF6_XSHUT = 8;
const int TOF7_XSHUT = 15;

// Navigation sensors
const int NAV_OUTER_LEFT  = 6;
const int NAV_INNER_LEFT  = 5;
const int NAV_INNER_RIGHT = 0;
const int NAV_OUTER_RIGHT = 7;

// Weight detection sensors
const int WEIGHT_LEFT_TOP     = 3;
const int WEIGHT_LEFT_BOTTOM  = 4;
const int WEIGHT_RIGHT_TOP    = 2;
const int WEIGHT_RIGHT_BOTTOM = 1;





static int tofDistances[8] = {
    0, 0, 0, 0,
    0, 0, 0, 0
};




static void shutdownAllToFs()
{
    for (int i = 0; i < 16; i++)
    {
        tofExpander.digitalWrite(i, LOW);
    }

    delay(100);
}

//code needed ot initialise purple tofs

static bool initialiseL0(
    VL53L0X &sensor,
    int xshutPin,
    int address,
    int number)
{
    tofExpander.digitalWrite(xshutPin, HIGH);

    delay(100);

    sensor.setTimeout(500);

    if (!sensor.init())
    {
        Serial.print("ERROR: L0 ToF ");
        Serial.print(number);
        Serial.println(" failed");

        tofExpander.digitalWrite(xshutPin, LOW);

        return false;
    }

    sensor.setAddress(address);

    sensor.startContinuous();

    Serial.print("L0 ToF ");
    Serial.print(number);
    Serial.print(" ready at 0x");
    Serial.println(address, HEX);

    return true;
}

//code needed to initialise black tofs

static bool initialiseL1(
    VL53L1X &sensor,
    int xshutPin,
    int address,
    int number)
{
    tofExpander.digitalWrite(xshutPin, HIGH);

    delay(100);

    sensor.setTimeout(500);

    if (!sensor.init())
    {
        Serial.print("ERROR: L1 ToF ");
        Serial.print(number);
        Serial.println(" failed");

        tofExpander.digitalWrite(xshutPin, LOW);

        return false;
    }

    sensor.setAddress(address);

    sensor.setDistanceMode(VL53L1X::Long);

    sensor.setMeasurementTimingBudget(50000);

    sensor.startContinuous(50);

    Serial.print("L1 ToF ");
    Serial.print(number);
    Serial.print(" ready at 0x");
    Serial.println(address, HEX);

    return true;
}



void tof_init()
{
    Wire.begin();

    Serial.println("Starting ToF setup...");


    // Initialise expander
    if (!tofExpander.begin(TOF_EXPANDER_ADDRESS))
    {
        Serial.println("ERROR: ToF expander not found");

        return;
    }

    Serial.println("ToF expander found");


    // Configure expander pins
    for (int i = 0; i < 16; i++)
    {
        tofExpander.pinMode(i, OUTPUT);
    }


    shutdownAllToFs();


    // Initialise sensors one at a time
    initialiseL0(tof0, TOF0_XSHUT, 0x30, 0);

    initialiseL1(tof1, TOF1_XSHUT, 0x31, 1);
    initialiseL1(tof2, TOF2_XSHUT, 0x32, 2);
    initialiseL1(tof3, TOF3_XSHUT, 0x33, 3);
    initialiseL1(tof4, TOF4_XSHUT, 0x34, 4);

    initialiseL0(tof5, TOF5_XSHUT, 0x35, 5);
    initialiseL0(tof6, TOF6_XSHUT, 0x36, 6);
    initialiseL0(tof7, TOF7_XSHUT, 0x37, 7);


    Serial.println("ToF setup complete");
}


//the black and purple tofs read in different ways hence why below there are two different commands
void tof_update()
{
    tofDistances[0] =
        tof0.readRangeContinuousMillimeters();

    tofDistances[1] =
        tof1.read();

    tofDistances[2] =
        tof2.read();

    tofDistances[3] =
        tof3.read();

    tofDistances[4] =
        tof4.read();

    tofDistances[5] =
        tof5.readRangeContinuousMillimeters();

    tofDistances[6] =
        tof6.readRangeContinuousMillimeters();

    tofDistances[7] =
        tof7.readRangeContinuousMillimeters();
}






void tof_print_readings(Stream &port)
{
    port.print("NAV: ");

    port.print("OL=");
    port.print(tof_get_nav_outer_left());

    port.print("  IL=");
    port.print(tof_get_nav_inner_left());

    port.print("  IR=");
    port.print(tof_get_nav_inner_right());

    port.print("  OR=");
    port.println(tof_get_nav_outer_right());


    port.print("WEIGHT: ");

    port.print("LT=");
    port.print(tof_get_weight_left_top());

    port.print("  LB=");
    port.print(tof_get_weight_left_bottom());

    port.print("  RT=");
    port.print(tof_get_weight_right_top());

    port.print("  RB=");
    port.println(tof_get_weight_right_bottom());
}

int tof_get_distance(int sensorNumber)
{
    return tofDistances[sensorNumber];
}




int tof_get_nav_outer_left()
{
    return tof_get_distance(NAV_OUTER_LEFT);
}


int tof_get_nav_inner_left()
{
    return tof_get_distance(NAV_INNER_LEFT);
}


int tof_get_nav_inner_right()
{
    return tof_get_distance(NAV_INNER_RIGHT);
}


int tof_get_nav_outer_right()
{
    return tof_get_distance(NAV_OUTER_RIGHT);
}

int tof_get_weight_left_top()
{
    return tof_get_distance(WEIGHT_LEFT_TOP);
}


int tof_get_weight_left_bottom()
{
    return tof_get_distance(WEIGHT_LEFT_BOTTOM);
}


int tof_get_weight_right_top()
{
    return tof_get_distance(WEIGHT_RIGHT_TOP);
}


int tof_get_weight_right_bottom()
{
    return tof_get_distance(WEIGHT_RIGHT_BOTTOM);
}
