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


const int NUM_TOF_SENSORS = 8;
const int TOF_FILTER_SIZE = 3;

const int NAV_TOF_MAX_MM = 800;
const int WEIGHT_TOF_MAX_MM = 600;



static int tofRawDistances[NUM_TOF_SENSORS];

static int tofFilterBuffer[NUM_TOF_SENSORS][TOF_FILTER_SIZE];

static int tofFilterIndex[NUM_TOF_SENSORS];

static int tofFilterCount[NUM_TOF_SENSORS];

static int tofFilteredDistances[NUM_TOF_SENSORS];

static bool tofReadingValid[NUM_TOF_SENSORS];




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

    for (int sensor = 0; sensor < NUM_TOF_SENSORS; sensor++)
    {
        tofRawDistances[sensor] = -1;
        tofFilteredDistances[sensor] = -1;

        tofFilterIndex[sensor] = 0;
        tofFilterCount[sensor] = 0;

        for (int i = 0; i < TOF_FILTER_SIZE; i++)
        {
            tofFilterBuffer[sensor][i] = -1;
        }
    }
    for (int sensor = 0; sensor < NUM_TOF_SENSORS; sensor++)
    {
        tofReadingValid[sensor] = false;
    }
    Serial.println("ToF setup complete");
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


//3 size buffer meidan filter used for ToFs

static int median3(int a,int b,int c)
{
    if (a > b)
    {
        int temp = a;
        a = b;
        b = temp;
    }

    if (b > c)
    {
        int temp = b;
        b = c;
        c = temp;
    }

    if (a > b)
    {
        int temp = a;
        a = b;
        b = temp;
    }

    return b;
}

static void updateMedianFilter(int sensor, int distance, int maxDistance)
{
    if (
        sensor < 0 ||
        sensor >= NUM_TOF_SENSORS
    )
    {
        return;
    }


    // Invalid / out-of-range reading
    if (
        distance <= 0 ||
        distance > maxDistance
    )
    {
        tofReadingValid[sensor] = false;

        return;
    }


    tofReadingValid[sensor] = true;


    tofFilterBuffer[sensor]
                   [tofFilterIndex[sensor]]
        = distance;


    tofFilterIndex[sensor]++;


    if (
        tofFilterIndex[sensor]
        >= TOF_FILTER_SIZE
    )
    {
        tofFilterIndex[sensor] = 0;
    }


    if (
        tofFilterCount[sensor]
        < TOF_FILTER_SIZE
    )
    {
        tofFilterCount[sensor]++;
    }


    if (tofFilterCount[sensor] == 1)
    {
        tofFilteredDistances[sensor] =
            tofFilterBuffer[sensor][0];

        return;
    }


    if (tofFilterCount[sensor] == 2)
    {
        int a =
            tofFilterBuffer[sensor][0];

        int b =
            tofFilterBuffer[sensor][1];

        tofFilteredDistances[sensor] =
            (a + b) / 2;

        return;
    }


    tofFilteredDistances[sensor] =
        median3(
            tofFilterBuffer[sensor][0],
            tofFilterBuffer[sensor][1],
            tofFilterBuffer[sensor][2]
        );
}


//the black and purple tofs read in different ways hence why below there are two different commands
void tof_update()
{
    int distance0 =  tof0.readRangeContinuousMillimeters();  
    tofRawDistances[0] = distance0;
    updateMedianFilter(0,distance0,NAV_TOF_MAX_MM);

    int distance1 = tof1.read();
    tofRawDistances[1] = distance1;
    updateMedianFilter(1,distance1, WEIGHT_TOF_MAX_MM);
    

    int distance2 = tof2.read();    
    tofRawDistances[2] = distance2;
    updateMedianFilter(2,distance2, NAV_TOF_MAX_MM);
    

    int distance3 = tof3.read();
    tofRawDistances[3] = distance3;
    updateMedianFilter(3,distance3, NAV_TOF_MAX_MM);
    

    int distance4 = tof4.read();
    tofRawDistances[4] = distance4;
    updateMedianFilter(4,distance4, NAV_TOF_MAX_MM);
    

    int distance5 = tof5.readRangeContinuousMillimeters();
    tofRawDistances[5] = distance5;
    updateMedianFilter(5,distance5,NAV_TOF_MAX_MM);
    

    int distance6 = tof6.readRangeContinuousMillimeters();
    tofRawDistances[6] = distance6;
    updateMedianFilter(6,distance6,NAV_TOF_MAX_MM);


    int distance7 = tof7.readRangeContinuousMillimeters();
    tofRawDistances[7] = distance7;
    updateMedianFilter(7,distance7, NAV_TOF_MAX_MM);
}



int tof_get_distance(
    int sensorNumber
)
{
    if (
        sensorNumber < 0 ||
        sensorNumber >= NUM_TOF_SENSORS
    )
    {
        return 0;
    }


    if (!tofReadingValid[sensorNumber])
    {
        return 0;
    }


    return tofFilteredDistances[
        sensorNumber
    ];
}

int tof_get_raw_distance(
    int sensorNumber
)
{
    if (
        sensorNumber < 0 ||
        sensorNumber >= NUM_TOF_SENSORS
    )
    {
        return -1;
    }

    return tofRawDistances[
        sensorNumber
    ];
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
