#include "ultrasound.h"

static const int LEFT_TRIG_PIN  = 25;
static const int LEFT_ECHO_PIN  = 14;

static const int RIGHT_TRIG_PIN = 27;
static const int RIGHT_ECHO_PIN = 15;

// Datasheet maximum range = 4.5 m
// Maximum round trip is approximately 26.5 ms.
// Give it a little margin.
static const uint32_t ECHO_TIMEOUT_US = 28000;

// Time between each snesor firing so they dont affect eachother with noise
static const uint32_t SENSOR_GAP_US = 30000;

volatile uint32_t left_echo_start_us = 0;
volatile uint32_t left_echo_width_us = 0;
volatile bool left_echo_ready = false;

volatile uint32_t right_echo_start_us = 0;
volatile uint32_t right_echo_width_us = 0;
volatile bool right_echo_ready = false;

static float left_distance_mm = -1.0f;
static float right_distance_mm = -1.0f;

//only want to be doing one thing at a time
enum UltrasoundState
{
    US_TRIGGER_LEFT,
    US_WAIT_LEFT,
    US_GAP_LEFT,

    US_TRIGGER_RIGHT,
    US_WAIT_RIGHT,
    US_GAP_RIGHT
};


static UltrasoundState ultrasound_state = US_TRIGGER_LEFT;

static uint32_t state_start_us = 0;


void left_echo_isr()
{
    if (digitalRead(LEFT_ECHO_PIN) == HIGH)
    {
        // Rising edge
        left_echo_start_us = micros();
    }
    else
    {
        // Falling edge
        left_echo_width_us = micros() - left_echo_start_us;
        left_echo_ready = true;
    }
}


void right_echo_isr()
{
    if (digitalRead(RIGHT_ECHO_PIN) == HIGH)
    {
        // Rising edge
        right_echo_start_us = micros();
    }
    else
    {
        // Falling edge
        right_echo_width_us = micros() - right_echo_start_us;
        right_echo_ready = true;
    }
}


static void trigger_left()
{
    left_echo_ready = false;
    //force to a known low state before sending trigger
    digitalWrite(LEFT_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(LEFT_TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(LEFT_TRIG_PIN, LOW);
}


static void trigger_right()
{
    right_echo_ready = false;
    //force to a known low state before sending trigger
    digitalWrite(RIGHT_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(RIGHT_TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(RIGHT_TRIG_PIN, LOW);
}

void ultrasound_init()
{
    pinMode(LEFT_TRIG_PIN, OUTPUT);
    pinMode(LEFT_ECHO_PIN, INPUT);

    pinMode(RIGHT_TRIG_PIN, OUTPUT);
    pinMode(RIGHT_ECHO_PIN, INPUT);

    digitalWrite(LEFT_TRIG_PIN, LOW);
    digitalWrite(RIGHT_TRIG_PIN, LOW);

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ECHO_PIN),
        left_echo_isr,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ECHO_PIN),
        right_echo_isr,
        CHANGE
    );

    ultrasound_state = US_TRIGGER_LEFT;

    Serial2.println("Ultrasound ready");
}

void ultrasound_exe()
{
    static unsigned long lastUltrasoundPrint = 0;

    // if (millis() - lastUltrasoundPrint >= 250)
    // {
    //     lastUltrasoundPrint = millis();

    //     Serial.print("US LEFT = ");
    //     Serial.print(ultrasound_get_left_mm());

    //     Serial.print(" mm   RIGHT = ");
    //     Serial.print(ultrasound_get_right_mm());

    //     Serial.println(" mm");
    // }
    uint32_t now = micros();

    switch (ultrasound_state)
    {
        
        case US_TRIGGER_LEFT:
        {
            trigger_left();

            state_start_us = micros();

            ultrasound_state = US_WAIT_LEFT;

            break;
        }


        
        case US_WAIT_LEFT:
        {
            if (left_echo_ready)
            {
                uint32_t pulse_width;

                noInterrupts();

                pulse_width = left_echo_width_us;
                left_echo_ready = false;

                interrupts();

                // Datasheet uses speed of sound = 340 m/s
                //
                // 340 m/s = 0.340 mm/us
                //
                // divide by 2 because ultrasound travels
                // to obstacle and back
                left_distance_mm =
                    pulse_width * 0.170f;

                state_start_us = now;

                ultrasound_state = US_GAP_LEFT;
            }
            else if (now - state_start_us >= ECHO_TIMEOUT_US)
            {
                // No valid echo
                left_distance_mm = -1.0f;

                state_start_us = now;

                ultrasound_state = US_GAP_LEFT;
            }

            break;
        }


        
        case US_GAP_LEFT:
        {
            if (now - state_start_us >= SENSOR_GAP_US)
            {
                ultrasound_state = US_TRIGGER_RIGHT;
            }

            break;
        }


       
        case US_TRIGGER_RIGHT:
        {
            trigger_right();

            state_start_us = micros();

            ultrasound_state = US_WAIT_RIGHT;

            break;
        }


        
        case US_WAIT_RIGHT:
        {
            if (right_echo_ready)
            {
                uint32_t pulse_width;

                noInterrupts();

                pulse_width = right_echo_width_us;
                right_echo_ready = false;

                interrupts();

                right_distance_mm =
                    pulse_width * 0.170f;

                state_start_us = now;

                ultrasound_state = US_GAP_RIGHT;
            }
            else if (now - state_start_us >= ECHO_TIMEOUT_US)
            {
                // No valid echo
                right_distance_mm = -1.0f;

                state_start_us = now;

                ultrasound_state = US_GAP_RIGHT;
            }

            break;
        }


        
        case US_GAP_RIGHT:
        {
            if (now - state_start_us >= SENSOR_GAP_US)
            {
                ultrasound_state = US_TRIGGER_LEFT;
            }

            break;
        }
    }
}




float ultrasound_get_left_mm()
{
    return left_distance_mm;
}


float ultrasound_get_right_mm()
{
    return right_distance_mm;
}

