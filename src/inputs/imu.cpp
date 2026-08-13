#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "imu.h"


// IMU OBJECT


// BNO055 is at I2C address 0x28.
//
static Adafruit_BNO055 bno =
    Adafruit_BNO055(55, 0x28, &Wire);

// IMU STATE


static bool imuOnline = false;

static float heading = 0.0;
static float pitch   = 0.0;
static float roll    = 0.0;


// Calibration scores:
// 0 = uncalibrated
// 3 = fully calibrated
static int calSystem = 0;
static int calGyro   = 0;
static int calAccel  = 0;
static int calMag    = 0;



// EEPROM CALIBRATION STORAGE


// EEPROM address used to store IMU calibration.
//
// If EEPROM is later used for other robot settings,
// reserve this region for the IMU.
const int IMU_EEPROM_ADDRESS = 0;


// Magic number lets us determine whether valid calibration
// has previously been saved.
const uint32_t IMU_CAL_MAGIC = 0xB055CA11;


// Prevent repeatedly writing calibration to EEPROM.
static bool calibrationSaved = false;
static bool calibrationNeeded = true;

// ============================================================
// LOAD SAVED CALIBRATION
// ============================================================

static bool loadCalibration()
{
    uint32_t magic = 0;

    EEPROM.get(IMU_EEPROM_ADDRESS, magic);

    if (magic != IMU_CAL_MAGIC)
    {
        Serial.println("No saved IMU calibration found");

        calibrationNeeded = true;

        return false;
    }

    adafruit_bno055_offsets_t offsets;

    EEPROM.get(
        IMU_EEPROM_ADDRESS + sizeof(magic),
        offsets
    );

    bno.setSensorOffsets(offsets);

    calibrationSaved = true;
    calibrationNeeded = false;

    Serial.println("Loaded saved IMU calibration from EEPROM");

    return true;
}


// ============================================================
// SAVE CALIBRATION
// ============================================================

static void saveCalibration()
{
    if (!calibrationNeeded)
    {
        return;
    }

    adafruit_bno055_offsets_t offsets;

    if (bno.getSensorOffsets(offsets))
    {
        EEPROM.put(
            IMU_EEPROM_ADDRESS,
            IMU_CAL_MAGIC
        );

        EEPROM.put(
            IMU_EEPROM_ADDRESS + sizeof(IMU_CAL_MAGIC),
            offsets
        );

        calibrationSaved = true;
        calibrationNeeded = false;

        Serial.println(
            "IMU fully calibrated - offsets saved to EEPROM"
        );
    }
}


// ============================================================
// INITIALISATION
// ============================================================

bool imu_init()
{
    Serial.println("Starting IMU setup...");


    // Wire.begin() is also called by the ToF module.
    // Calling it here allows this module to work independently.
    Wire.begin();


    imuOnline = bno.begin();


    if (!imuOnline)
    {
        Serial.println("ERROR: BNO055 IMU not detected");

        return false;
    }


    Serial.println("BNO055 detected");


    // BNO055 needs a short delay after begin().
    delay(1000);


    // Attempt to restore previous calibration.
    loadCalibration();


    Serial.println("IMU setup complete");

    return true;
}



// UPDATE IMU


void imu_update()
{
    if (!imuOnline)
    {
        return;
    }


    
    // Read fused orientation
    

    imu::Vector<3> euler =
        bno.getVector(Adafruit_BNO055::VECTOR_EULER);


    heading = euler.x();
    pitch   = euler.y();
    roll    = euler.z();


    
    // Read calibration status
    
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;


    bno.getCalibration(
        &system,
        &gyro,
        &accel,
        &mag
    );


    calSystem = system;
    calGyro   = gyro;
    calAccel  = accel;
    calMag    = mag;


    // If this is the first calibration, save it once complete.
    if (calibrationNeeded)
    {
        saveCalibration();
    }
}


// ORIENTATION GETTERS


float imu_get_heading()
{
    return heading;
}


float imu_get_pitch()
{
    return pitch;
}


float imu_get_roll()
{
    return roll;
}


// CALIBRATION GETTER


void imu_get_calibration(
    int &system,
    int &gyro,
    int &accel,
    int &mag
)
{
    system = calSystem;
    gyro   = calGyro;
    accel  = calAccel;
    mag    = calMag;
}



// SENSOR STATUS


bool imu_is_online()
{
    return imuOnline;
}



// DEBUG PRINT


void imu_print_readings()
{
    if (!imuOnline)
    {
        Serial.println("IMU: OFFLINE");

        return;
    }


    Serial.print("Heading: ");
    Serial.print(heading, 1);

    Serial.print("   Pitch: ");
    Serial.print(pitch, 1);

    Serial.print("\n   Roll: ");
    Serial.print(roll, 1);

}


// ============================================================
// CLEAR SAVED CALIBRATION
// ============================================================

void imu_clear_calibration()
{
    uint32_t emptyMagic = 0;

    EEPROM.put(
        IMU_EEPROM_ADDRESS,
        emptyMagic
    );


    calibrationSaved = false;


    Serial.println(
        "Saved IMU calibration cleared - restart and recalibrate"
    );
}