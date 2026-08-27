#include "inputs/encoders.h"


// ============================================================
// ENCODER PINS
// ============================================================

// DIGITAL RAW2 connector
//
// Left encoder:
//     A -> D2
//     B -> D3
//
// Right encoder:
//     A -> D4
//     B -> D5

const int LEFT_ENCODER_A_PIN  = 2;
const int LEFT_ENCODER_B_PIN  = 3;

const int RIGHT_ENCODER_A_PIN = 4;
const int RIGHT_ENCODER_B_PIN = 5;


// If forward motion produces a negative count on one side,
// change that side from +1 to -1.
const int LEFT_ENCODER_SIGN  = -1;
const int RIGHT_ENCODER_SIGN = 1;



// ENCODER COUNTS


static volatile long leftRawCount = 0;
static volatile long rightRawCount = 0;



// DISTANCE CALIBRATION

static float leftMmPerCount = 1000.00f/6400.00f;
static float rightMmPerCount = 1000.00f/6400.00f;


// ============================================================
// INTERRUPTS
// ============================================================

static void leftEncoderISR()
{
    bool a = digitalRead(LEFT_ENCODER_A_PIN);
    bool b = digitalRead(LEFT_ENCODER_B_PIN);

    if (a == b)
    {
        leftRawCount++;
    }
    else
    {
        leftRawCount--;
    }
}


static void rightEncoderISR()
{
    bool a = digitalRead(RIGHT_ENCODER_A_PIN);
    bool b = digitalRead(RIGHT_ENCODER_B_PIN);

    if (a == b)
    {
        rightRawCount++;
    }
    else
    {
        rightRawCount--;
    }
}


// ============================================================
// INITIALISE
// ============================================================

bool encoders_init()
{
    pinMode(
        LEFT_ENCODER_A_PIN,
        INPUT
    );

    pinMode(
        LEFT_ENCODER_B_PIN,
        INPUT
    );

    pinMode(
        RIGHT_ENCODER_A_PIN,
        INPUT
    );

    pinMode(
        RIGHT_ENCODER_B_PIN,
        INPUT
    );


    attachInterrupt(
        digitalPinToInterrupt(
            LEFT_ENCODER_A_PIN
        ),
        leftEncoderISR,
        CHANGE
    );


    attachInterrupt(
        digitalPinToInterrupt(
            RIGHT_ENCODER_A_PIN
        ),
        rightEncoderISR,
        CHANGE
    );


    encoders_reset();


    Serial.println("Encoders ready");

    return true;
}


// ============================================================
// COUNTS
// ============================================================

long encoders_get_left_count()
{
    noInterrupts();

    long count = leftRawCount;

    interrupts();


    return count * LEFT_ENCODER_SIGN;
}


long encoders_get_right_count()
{
    noInterrupts();

    long count = rightRawCount;

    interrupts();


    return count * RIGHT_ENCODER_SIGN;
}


void encoders_reset()
{
    noInterrupts();

    leftRawCount = 0;
    rightRawCount = 0;

    interrupts();
}


// ============================================================
// DISTANCE CALIBRATION
// ============================================================

void encoders_set_mm_per_count(
    float leftValue,
    float rightValue
)
{
    leftMmPerCount = leftValue;
    rightMmPerCount = rightValue;
}


bool encoders_is_calibrated()
{
    return (
        leftMmPerCount > 0.0 &&
        rightMmPerCount > 0.0
    );
}


float encoders_get_left_mm_per_count()
{
    return leftMmPerCount;
}


float encoders_get_right_mm_per_count()
{
    return rightMmPerCount;
}


float encoders_get_left_distance_mm()
{
    return
        encoders_get_left_count()
        * leftMmPerCount;
}


float encoders_get_right_distance_mm()
{
    return
        encoders_get_right_count()
        * rightMmPerCount;
}


// ============================================================
// DEBUG
// ============================================================

void encoders_print(Stream &port)
{
    port.print("ENC L: ");
    port.print(
        encoders_get_left_count()
    );

    port.print("   R: ");
    port.println(
        encoders_get_right_count()
    );
}