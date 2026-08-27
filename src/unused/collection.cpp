#include "collection.h"

#include <Arduino.h>

#include "driving_controller.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"


enum CollectionState
{
    COLLECTION_IDLE,

    COLLECTION_SETTLING,

    COLLECTION_LOWERING,

    COLLECTION_GRABBING,

    COLLECTION_LIFTING,

    COLLECTION_RELEASING,

    COLLECTION_RETURNING
};


static CollectionState collectionState = COLLECTION_IDLE;

static unsigned long stateStartTime = 0;


// ============================================================
// TIMINGS
// ============================================================

// These are just starting values.
// Tune them from physical testing.

const unsigned long SETTLE_TIME_MS  = 1500;
const unsigned long LOWER_TIME_MS   = 2400;
const unsigned long GRAB_TIME_MS    = 600;
const unsigned long LIFT_TIME_MS    = 1800;
const unsigned long RELEASE_TIME_MS = 800;
const unsigned long RETURN_TIME_MS  = 800;


// ============================================================
// INTERNAL HELPER
// ============================================================

static void changeState(CollectionState newState)
{
    collectionState = newState;
    stateStartTime = millis();
}


// ============================================================
// INITIALISATION
// ============================================================

void collection_init()
{
    collectionState = COLLECTION_IDLE;
}


// ============================================================
// START COLLECTION
// ============================================================

void collection_start()
{
    // Ignore another request while already collecting
    if (collectionState != COLLECTION_IDLE)
    {
        return;
    }

    Serial.println("Collection started");
    Serial2.println("Collection started");

    // Stop chassis before operating crane
    motor_control_stop();

    changeState(
        COLLECTION_SETTLING
    );
}


// ============================================================
// COLLECTION EXECUTION
// ============================================================

void collection_exe()
{
    unsigned long elapsed = millis() - stateStartTime;


    switch (collectionState)
    {
        // ----------------------------------------------------
        case COLLECTION_IDLE:
            break;


        // ----------------------------------------------------
        case COLLECTION_SETTLING:

            if (elapsed >= SETTLE_TIME_MS)
            {
                Serial.println("Lowering crane");

                craneVertical();

                changeState(COLLECTION_LOWERING);
            }

            break;


        // ----------------------------------------------------
        case COLLECTION_LOWERING:

            if (elapsed >= LOWER_TIME_MS)
            {
                Serial.println("Magnet ON");

                emag_on();

                changeState(COLLECTION_GRABBING);
            }

            break;


        // ----------------------------------------------------
        case COLLECTION_GRABBING:

            if (elapsed >= GRAB_TIME_MS)
            {
                Serial.println("Lifting weight");

                craneDrop();

                changeState(COLLECTION_LIFTING);
            }

            break;


        // ----------------------------------------------------
        case COLLECTION_LIFTING:

            if (elapsed >= LIFT_TIME_MS)
            {
                Serial.println("Magnet OFF");

                emag_off();

                changeState(COLLECTION_RELEASING);
            }

            break;


        // ----------------------------------------------------
        case COLLECTION_RELEASING:

            if (elapsed >= RELEASE_TIME_MS)
            {
                Serial.println("Returning crane");

                craneIdle();

                changeState(COLLECTION_RETURNING);
            }

            break;


        // ----------------------------------------------------
        case COLLECTION_RETURNING:

            if (elapsed >= RETURN_TIME_MS)
            {
                Serial.println("Collection complete");

                changeState(COLLECTION_IDLE);
            }

            break;
    }
}


// ============================================================
// STATUS
// ============================================================

bool collectionIsActive()
{
    return collectionState != COLLECTION_IDLE;
}