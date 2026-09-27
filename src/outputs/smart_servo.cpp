#include <Arduino.h>
#include <HerkulexServo.h>

#include "smart_servo.h"


static const uint32_t SERVO_BAUD = 115200;



// Smart servo is connected to Teensy Serial1
static HerkulexServoBus herkulexBus(
    Serial1
);

static const uint8_t LEFT_ARM_ID  = 4;
static const uint8_t RIGHT_ARM_ID = 1;

// Servo object is created once we know which ID to use
static HerkulexServo* leftArmServo  = nullptr;
static HerkulexServo* rightArmServo = nullptr;

static uint8_t smartServoId = 0;


static const uint16_t LEFT_ARM_OPEN_POSITION   = 360;
static const uint16_t LEFT_ARM_CLOSED_POSITION = 614;

static const uint16_t RIGHT_ARM_OPEN_POSITION   = 800;
static const uint16_t RIGHT_ARM_CLOSED_POSITION = 500;

static const uint8_t ARM_PLAYTIME = 80;





bool smartservo_init()
{
    Serial1.begin(SERVO_BAUD);

    delay(100);

    leftArmServo = new HerkulexServo(
        herkulexBus,
        LEFT_ARM_ID
    );

    rightArmServo = new HerkulexServo(
        herkulexBus,
        RIGHT_ARM_ID
    );

    Serial.print("Left arm servo ID: ");
    Serial.println(LEFT_ARM_ID);

    Serial.print("Right arm servo ID: ");
    Serial.println(RIGHT_ARM_ID);

    return true;
}



void smartservo_update()
{
    herkulexBus.update();
}



void smartservo_torque_on()
{
    if (leftArmServo != nullptr)
    {
        leftArmServo->setTorqueOn();
    }

    if (rightArmServo != nullptr)
    {
        rightArmServo->setTorqueOn();
    }
}


void smartservo_torque_off()
{
    if (leftArmServo != nullptr)
    {
        leftArmServo->setTorqueOff();
    }

    if (rightArmServo != nullptr)
    {
        rightArmServo->setTorqueOff();
    }
}


void smartservo_arms_open()
{
    if (leftArmServo != nullptr)
    {
        leftArmServo->setTorqueOn();
        leftArmServo->setPosition(
            LEFT_ARM_OPEN_POSITION,
            ARM_PLAYTIME
        );
    }

    if (rightArmServo != nullptr)
    {
        rightArmServo->setTorqueOn();
        rightArmServo->setPosition(
            RIGHT_ARM_OPEN_POSITION,
            ARM_PLAYTIME
        );
    }

    Serial.println("Front arms opening");
}


void smartservo_arms_close()
{
    if (leftArmServo != nullptr)
    {
        leftArmServo->setTorqueOn();
        leftArmServo->setPosition(
            LEFT_ARM_CLOSED_POSITION,
            ARM_PLAYTIME
        );
    }

    if (rightArmServo != nullptr)
    {
        rightArmServo->setTorqueOn();
        rightArmServo->setPosition(
            RIGHT_ARM_CLOSED_POSITION,
            ARM_PLAYTIME
        );
    }

    Serial.println("Front arms closing");
}

static HerkulexServo* getServoById(uint8_t servoId)
{
    if (servoId == LEFT_ARM_ID)
    {
        return leftArmServo;
    }

    if (servoId == RIGHT_ARM_ID)
    {
        return rightArmServo;
    }

    return nullptr;
}


void smartservo_set_position(
    uint8_t servoId,
    uint16_t position,
    uint8_t playtime
)
{
    HerkulexServo* servo = getServoById(servoId);

    if (servo == nullptr)
    {
        Serial.print("Invalid smart servo ID: ");
        Serial.println(servoId);
        return;
    }

    if (position > 1023)
    {
        position = 1023;
    }

    servo->setTorqueOn();

    servo->setPosition(
        position,
        playtime
    );
}


uint16_t smartservo_get_position(
    uint8_t servoId
)
{
    HerkulexServo* servo = getServoById(servoId);

    if (servo == nullptr)
    {
        return 0;
    }

    return servo->getPosition();
}

void smartservo_print_status(
    uint8_t servoId
)
{
    HerkulexServo* servo = getServoById(servoId);

    if (servo == nullptr)
    {
        Serial.print("Invalid smart servo ID: ");
        Serial.println(servoId);
        return;
    }

    HerkulexStatusError statusError;
    HerkulexStatusDetail statusDetail;

    servo->getStatus(
        statusError,
        statusDetail
    );

    uint16_t position =
        servo->getPosition();

    Serial.print("Smart servo ID: ");
    Serial.print(servoId);

    Serial.print("   Position: ");
    Serial.print(position);

    Serial.print("   Error: 0x");
    Serial.print(
        static_cast<uint8_t>(statusError),
        HEX
    );

    Serial.print("   Detail: 0x");
    Serial.println(
        static_cast<uint8_t>(statusDetail),
        HEX
    );
}




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


void smartservo_test_left(uint16_t position)
{
    if (leftArmServo == nullptr)
    {
        Serial.println("Left servo not initialised");
        return;
    }

    position = constrain(position, 0, 1023);

    leftArmServo->setTorqueOn();
    leftArmServo->setPosition(position, 80);

    Serial.print("Left servo -> ");
    Serial.println(position);
}


void smartservo_test_right(uint16_t position)
{
    if (rightArmServo == nullptr)
    {
        Serial.println("Right servo not initialised");
        return;
    }

    position = constrain(position, 0, 1023);

    rightArmServo->setTorqueOn();
    rightArmServo->setPosition(position, 80);

    Serial.print("Right servo -> ");
    Serial.println(position);
}

void smartservo_print_positions()
{
    if (leftArmServo != nullptr)
    {
        Serial2.print("Left position: ");
        Serial2.println(leftArmServo->getPosition());
    }

    if (rightArmServo != nullptr)
    {
        Serial2.print("Right position: ");
        Serial2.println(rightArmServo->getPosition());
    }
}
