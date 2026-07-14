#include <Wire.h>
#include <SparkFunSX1509.h> // SparkFun SX1509 I/O Expander library, v2.0.1
#include <Servo.h> 


Servo myservo;
int pos = 0;
const byte SX1509_LIMIT_ADDRESS = 0x3E;

const byte AIO6_PIN = 6;
const byte EMAG_PIN = 26;
const byte PROX_PIN = 20;
const byte PROX_PIN_2 = 14;


String state = "desc";
SX1509 io;

void setup() {
  Serial.begin(115200);
  while (!Serial) { } 

  if (!io.begin(SX1509_LIMIT_ADDRESS)) {
    Serial.println("Failed to find SX1509 at 0x3E - check wiring/I2C cable");
    while (1) { }
  }

  io.pinMode(AIO6_PIN, INPUT_PULLUP);

  Serial.println("SX1509 limit switch expander ready");

  myservo.attach(1, 500, 2500);
  pinMode(EMAG_PIN, OUTPUT);
  pinMode(PROX_PIN_2, INPUT);
  pinMode(PROX_PIN, INPUT);

}

void loop() {
  Serial.print(state);
  Serial.print(" ");
  Serial.print(io.digitalRead(AIO6_PIN) == LOW);
  Serial.print(" ");
  Serial.println(analogRead(PROX_PIN) > 500);
  Serial.println(analogRead(PROX_PIN_2));

  if (state == "desc") {
      myservo.write(20);
      delay(2000);
      bool pressed = (io.digitalRead(AIO6_PIN) == LOW);
      bool metal = (analogRead(PROX_PIN) > 500);
      if (metal) {
        state = "inc";
      } else if (!pressed ) {


        state = "absent";
      } else {
        state = "stationary";
      }

  } else if (state == "inc") {
      digitalWrite(EMAG_PIN, HIGH);
      delay(200);
      myservo.write(100);
      delay(2000);
      state = "dropping";

  } else if (state == "absent") {
    myservo.write(0);
    delay(2000);
    bool pressed = (io.digitalRead(AIO6_PIN) == LOW);
    bool metal = (analogRead(PROX_PIN) > 500);
    if (metal) {
      state = "inc";
    } else {
      state = "stationary";
    }

  } else if (state == "stationary") {
    delay(1000);
    digitalWrite(EMAG_PIN, LOW);

  } else if (state == "dropping") {
    digitalWrite(EMAG_PIN, LOW);
    delay(1000);
    state = "desc";

  }

}
