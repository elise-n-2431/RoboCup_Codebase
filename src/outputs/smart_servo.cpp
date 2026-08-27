#include <Arduino.h>
#include <HerkulexServo.h>

#include "smart_servo.h"



static const uint16_t GATE_CLOSED_POSITION = 780;
static const uint16_t GATE_OPEN_POSITION   = 400;

static const uint8_t GATE_PLAYTIME = 80;



static const uint32_t SERVO_BAUD = 115200;



// Smart servo is connected to Teensy Serial1
static HerkulexServoBus herkulexBus(
    Serial1
);


// Servo object is created once we know which ID to use
static HerkulexServo* smartServo = nullptr;

static uint8_t smartServoId = 0;





bool smartservo_init(
    uint8_t servoId
)
{
    Serial1.begin(
        SERVO_BAUD
    );


    delay(100);


    smartServoId = servoId;


    smartServo = new HerkulexServo(
        herkulexBus,
        smartServoId
    );


    Serial.print(
        "Smart servo initialised with ID "
    );

    Serial.println(
        smartServoId
    );


    // Do NOT enable torque automatically.
    // This avoids unexpected motion at startup.

    return true;
}


// ============================================================
// UPDATE
// ============================================================

void smartservo_update()
{
    herkulexBus.update();
}


// ============================================================
// TORQUE
// ============================================================

void smartservo_torque_on()
{
    if (smartServo == nullptr)
    {
        return;
    }


    smartServo->setTorqueOn();
}


void smartservo_torque_off()
{
    if (smartServo == nullptr)
    {
        return;
    }


    smartServo->setTorqueOff();
}


// ============================================================
// POSITION
// ============================================================

void smartservo_set_position(
    uint16_t position,
    uint8_t playtime
)
{
    if (smartServo == nullptr)
    {
        return;
    }


    if (position > 1023)
    {
        position = 1023;
    }


    smartServo->setPosition(
        position,
        playtime
    );
}


// ============================================================
// GET POSITION
// ============================================================

uint16_t smartservo_get_position()
{
    if (smartServo == nullptr)
    {
        return 0;
    }


    return smartServo->getPosition();
}


// ============================================================
// STATUS
// ============================================================

void smartservo_print_status()
{
    if (smartServo == nullptr)
    {
        Serial.println(
            "Smart servo not initialised"
        );

        return;
    }


    HerkulexStatusError statusError;
    HerkulexStatusDetail statusDetail;


    smartServo->getStatus(
        statusError,
        statusDetail
    );


    uint16_t position =
        smartServo->getPosition();


    Serial.print(
        "Smart servo ID: "
    );

    Serial.print(
        smartServoId
    );


    Serial.print(
        "   Position: "
    );

    Serial.print(
        position
    );


    Serial.print(
        "   Error: 0x"
    );

    Serial.print(
        static_cast<uint8_t>(
            statusError
        ),
        HEX
    );


    Serial.print(
        "   Detail: 0x"
    );

    Serial.println(
        static_cast<uint8_t>(
            statusDetail
        ),
        HEX
    );
}


// ============================================================
// SCAN
// ============================================================

void smartservo_scan()
{
    Serial.println(
        "Scanning for HerkuleX servos..."
    );


    int found = 0;


    for (
        uint16_t id = 0;
        id <= 253;
        id++
    )
    {
        HerkulexPacket response;


        if (
            herkulexBus.sendPacketAndReadResponse(
                response,
                id,
                HerkulexCommand::Stat
            )
        )
        {
            Serial.print(
                "Found servo ID: "
            );

            Serial.println(
                id
            );

            found++;
        }
    }


    Serial.print(
        "Scan complete. Found "
    );

    Serial.print(
        found
    );

    Serial.println(
        " servo(s)"
    );
}

bool smartservo_ping()
{
    HerkulexPacket response;

    bool responded =
        herkulexBus.sendPacketAndReadResponse(
            response,
            smartServoId,
            HerkulexCommand::Stat
        );


    if (responded)
    {
        Serial.print(
            "Servo responded at ID "
        );

        Serial.println(
            smartServoId
        );

        return true;
    }


    Serial.print(
        "NO response from servo ID "
    );

    Serial.println(
        smartServoId
    );

    return false;
}


void smartservo_gate_open()
{
    if (smartServo == nullptr)
    {
        return;
    }

    smartServo->setTorqueOn();

    smartServo->setPosition(
        GATE_OPEN_POSITION,
        GATE_PLAYTIME
    );

    Serial.println("Gate opening");
}


void smartservo_gate_close()
{
    if (smartServo == nullptr)
    {
        return;
    }

    smartServo->setTorqueOn();

    smartServo->setPosition(
        GATE_CLOSED_POSITION,
        GATE_PLAYTIME
    );

    Serial.println("Gate closing");
}
