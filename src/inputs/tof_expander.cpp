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
static VL53L0X tof8;

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
const int TOF8_XSHUT = 9;

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
const int WEIGHT_MIDDLE = 8;


const int NUM_TOF_SENSORS = 9;
const int TOF_FILTER_SIZE = 3;

const int NAV_TOF_MAX_MM = 1200;
const int WEIGHT_TOF_MAX_MM = 800;
const unsigned long TOF_STALE_MS = 250;


static int tofRawDistances[NUM_TOF_SENSORS];

static int tofFilterBuffer[NUM_TOF_SENSORS][TOF_FILTER_SIZE];

static int tofFilterIndex[NUM_TOF_SENSORS];

static int tofFilterCount[NUM_TOF_SENSORS];

static int tofFilteredDistances[NUM_TOF_SENSORS];
static int tofRangeStatus[NUM_TOF_SENSORS];

static bool tofOnline[NUM_TOF_SENSORS];
static bool tofReadingValid[NUM_TOF_SENSORS];
static bool tofNoReturn[NUM_TOF_SENSORS];

static unsigned long tofLastReadingAt[NUM_TOF_SENSORS];
static uint32_t tofSampleNumber[NUM_TOF_SENSORS];


static void shutdownAllToFs()
{
    for (int i = 0; i < 16; i++)
    {
        tofExpander.digitalWrite(i, LOW);
    }

    delay(100);
}

//code needed ot initialise purple tofs

static bool initialiseL0(VL53L0X &sensor, int xshutPin, int address, int number)
{
    tofExpander.digitalWrite(xshutPin, HIGH);
    delay(100);
    sensor.setTimeout(1000);

    if (!sensor.init()) {
        Serial.print("ERROR: L0 ToF ");
        Serial.print(number);
        Serial.println(" failed");

        tofExpander.digitalWrite(xshutPin, LOW);
        return false;
    }

    sensor.setAddress(address);
    sensor.startContinuous();
    sensor.setTimeout(5);

    Serial.print("L0 ToF ");
    Serial.print(number);
    Serial.print(" ready at 0x");
    Serial.println(address, HEX);

    return true;
}

//code needed to initialise black tofs

static bool initialiseL1(VL53L1X &sensor, int xshutPin, int address, int number)
{
    tofExpander.digitalWrite(xshutPin, HIGH);
    delay(100);
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.print("ERROR: L1 ToF ");
        Serial.print(number);
        Serial.println(" failed");

        tofExpander.digitalWrite(xshutPin, LOW);
        return false;
    }

    sensor.setAddress(address);
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setROISize(8, 8);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);
    sensor.setTimeout(5);

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

    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        tofOnline[i] = tofReadingValid[i] = tofNoReturn[i] = false;
        tofRawDistances[i] = tofFilteredDistances[i] = tofRangeStatus[i] = -1;
        tofFilterIndex[i] = tofFilterCount[i] = 0;
        tofLastReadingAt[i] = tofSampleNumber[i] = 0;
    }

    if (!tofExpander.begin(TOF_EXPANDER_ADDRESS)) {
        Serial.println("ERROR: ToF expander not found");
        return;
    }

    for (int i = 0; i < 16; i++) tofExpander.pinMode(i, OUTPUT);
    shutdownAllToFs();

    tofOnline[0] = initialiseL0(tof0, TOF0_XSHUT, 0x30, 0);
    tofOnline[1] = initialiseL1(tof1, TOF1_XSHUT, 0x31, 1);
    tofOnline[2] = initialiseL1(tof2, TOF2_XSHUT, 0x32, 2);
    tofOnline[3] = initialiseL1(tof3, TOF3_XSHUT, 0x33, 3);
    tofOnline[4] = initialiseL1(tof4, TOF4_XSHUT, 0x34, 4);
    tofOnline[5] = initialiseL0(tof5, TOF5_XSHUT, 0x35, 5);
    tofOnline[6] = initialiseL0(tof6, TOF6_XSHUT, 0x36, 6);
    tofOnline[7] = initialiseL0(tof7, TOF7_XSHUT, 0x37, 7);
    tofOnline[8] = initialiseL0(tof8, TOF8_XSHUT, 0x38, 8);

    Serial.println("ToF setup complete");
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


static void updateMedianFilter(int number, int distance, int maxDistance)
{
    if (distance <= 0) {
        tofFilterIndex[number] = tofFilterCount[number] = 0;
        return;
    }

    distance = min(distance, maxDistance);

    tofFilterBuffer[number][tofFilterIndex[number]] = distance;
    tofFilterIndex[number] = (tofFilterIndex[number] + 1) % TOF_FILTER_SIZE;
    tofFilterCount[number] = min(tofFilterCount[number] + 1, TOF_FILTER_SIZE);

    if (tofFilterCount[number] == 1) {
        tofFilteredDistances[number] = distance;
    } else if (tofFilterCount[number] == 2) {
        tofFilteredDistances[number] = min(tofFilterBuffer[number][0],
                                           tofFilterBuffer[number][1]);
    } else {
        tofFilteredDistances[number] = median3(tofFilterBuffer[number][0],
                                              tofFilterBuffer[number][1],
                                              tofFilterBuffer[number][2]);
    }
}


static void saveReading(int number, int distance, int maxDistance)
{
    // Positive = distance, zero = fresh weak return, negative = unusable.
    if (millis() - tofLastReadingAt[number] > TOF_STALE_MS) {
        tofFilterIndex[number] = tofFilterCount[number] = 0;
    }

    tofReadingValid[number] = distance > 0;
    tofNoReturn[number] = distance == 0;
    tofLastReadingAt[number] = millis();
    tofSampleNumber[number]++;

    updateMedianFilter(number, distance, maxDistance);
}



static void updateL0(VL53L0X &sensor, int number, int maxDistance)
{
    if (!tofOnline[number]) return;

    uint8_t ready = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);

    if (sensor.last_status != 0) {
        saveReading(number, -1, maxDistance);
        return;
    }

    if (!(ready & 7)) return;

    int status = (sensor.readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x78) >> 3;

    if (sensor.last_status != 0) {
        saveReading(number, -1, maxDistance);
        return;
    }

    int distance = sensor.readRangeContinuousMillimeters();

    tofRawDistances[number] = distance;
    tofRangeStatus[number] = status;

    if (sensor.timeoutOccurred() || sensor.last_status != 0) {
        saveReading(number, -1, maxDistance);
    } else if (status == 11 && distance > 0 && distance < 8190) {
        // L0 device status 11: range measurement completed.
        saveReading(number, distance, maxDistance);
    } else if (status == 4) {
        // L0 device status 4: insufficient return signal.
        saveReading(number, 0, maxDistance);
    } else {
        saveReading(number, -1, maxDistance);
    }
}

static void updateL1(VL53L1X &sensor, int number, int maxDistance)
{
    if (!tofOnline[number]) return;

    bool ready = sensor.dataReady();

    if (sensor.last_status != 0) {
        saveReading(number, -1, maxDistance);
        return;
    }

    if (!ready) return;

    int distance = sensor.read(false);
    VL53L1X::RangeStatus status = sensor.ranging_data.range_status;

    tofRawDistances[number] = distance;
    tofRangeStatus[number] = status;

    if (sensor.last_status != 0) {
        saveReading(number, -1, maxDistance);
    } else if (status == VL53L1X::RangeValid && distance > 0) {
        saveReading(number, distance, maxDistance);
    } else if (status == VL53L1X::RangeValidMinRangeClipped) {
        saveReading(number, max(distance, 1), maxDistance);
    } else if (status == VL53L1X::SignalFail) {
        saveReading(number, 0, maxDistance);
    } else {
        saveReading(number, -1, maxDistance);
    }
}

void tof_update()
{
    static unsigned long lastPollAt = 0;

    if (millis() - lastPollAt < 5) return;
    lastPollAt = millis();

    updateL0(tof0, 0, NAV_TOF_MAX_MM);
    updateL1(tof1, 1, WEIGHT_TOF_MAX_MM);
    updateL1(tof2, 2, WEIGHT_TOF_MAX_MM);
    updateL1(tof3, 3, WEIGHT_TOF_MAX_MM);
    updateL1(tof4, 4, WEIGHT_TOF_MAX_MM);
    updateL0(tof5, 5, NAV_TOF_MAX_MM);
    updateL0(tof6, 6, NAV_TOF_MAX_MM);
    updateL0(tof7, 7, NAV_TOF_MAX_MM);
    updateL0(tof8, 8, NAV_TOF_MAX_MM);
}



int tof_get_distance(int number)
{
    if (number < 0 || number >= NUM_TOF_SENSORS) return -1;

    if (!tofOnline[number] ||
        millis() - tofLastReadingAt[number] > TOF_STALE_MS) return -1;

    if (tofNoReturn[number]) return 0;
    if (!tofReadingValid[number]) return -1;

    bool navigationSensor = number == NAV_OUTER_LEFT || number == NAV_INNER_LEFT ||
                            number == NAV_INNER_RIGHT || number == NAV_OUTER_RIGHT;

    // React immediately to a closer obstacle; filter increases in clearance.
    if (navigationSensor) return min(tofRawDistances[number], tofFilteredDistances[number]);

    return tofFilteredDistances[number];
}


int tof_get_raw_distance(int number)
{
    return number >= 0 && number < NUM_TOF_SENSORS ? tofRawDistances[number] : -1;
}

uint32_t tof_get_sample_number(int number)
{
    return number >= 0 && number < NUM_TOF_SENSORS ? tofSampleNumber[number] : 0;
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


int tof_get_weight_middle()
{
    return tof_get_distance(WEIGHT_MIDDLE);
}

int tof_get_weight_right_bottom()
{
    return tof_get_distance(WEIGHT_RIGHT_BOTTOM);
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

    port.print("  MIDDLE =");
    port.println(tof_get_weight_middle());
}


