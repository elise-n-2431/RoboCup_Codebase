#include "lucas_driver.h"
#include <Arduino.h>
#include <Servo.h>

Servo motorLeft;
Servo motorRight;

// Old double DC motor pins
const int LEFT_PIN  = 0;   // D0
const int RIGHT_PIN = 1;   // D1

const int STOP_US = 1500;
const int FULL_FORWARD_US = 1950;
const int FULL_REVERSE_US = 1050;

int drivePower = 250;      // Start gentle
const int MIN_POWER = 80;
const int MAX_POWER = 430;
const int POWER_STEP = 25;

// Your old code used left=1950 and right=1050 for both forward,
// so assume right motor is inverted.
const bool LEFT_INVERTED = false;
const bool RIGHT_INVERTED = true;

bool motorsArmed = false;
String lineBuffer = "";

int clampPulse(int us)
{
  if (us > FULL_FORWARD_US) return FULL_FORWARD_US;
  if (us < FULL_REVERSE_US) return FULL_REVERSE_US;
  return us;
}

void writeMotors(int leftUs, int rightUs)
{
  motorLeft.writeMicroseconds(clampPulse(leftUs));
  motorRight.writeMicroseconds(clampPulse(rightUs));
}

void stopMotors()
{
  writeMotors(STOP_US, STOP_US);
}

void driveTracks(int leftTrack, int rightTrack)
{
  if (!motorsArmed) {
    stopMotors();
    Serial2.println("Motors OFF. Send mon first.");
    return;
  }

  int leftSign = LEFT_INVERTED ? -leftTrack : leftTrack;
  int rightSign = RIGHT_INVERTED ? -rightTrack : rightTrack;

  int leftPulse = STOP_US + leftSign * drivePower;
  int rightPulse = STOP_US + rightSign * drivePower;

  writeMotors(leftPulse, rightPulse);
}

void armMotors()
{
  motorsArmed = true;
  stopMotors();
  Serial.println("motors ON");
  Serial2.println("motors ON");
}

void disarmMotors()
{
  stopMotors();
  motorsArmed = false;
  Serial.println("motors OFF");
  Serial2.println("motors OFF");
}

void printHelp()
{
  Serial2.println();
  Serial2.println("Commands:");
  Serial2.println("mon  = motors on");
  Serial2.println("moff = motors off");
  Serial2.println("w    = forward");
  Serial2.println("s    = reverse");
  Serial2.println("a    = turn left");
  Serial2.println("d    = turn right");
  Serial2.println("x    = stop");
  Serial2.println("+/-  = speed up/down");
  Serial2.print("speed offset = ");
  Serial2.println(drivePower);
  Serial2.println();
}

void processLine(String cmd)
{
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "mon") {
    armMotors();
  } else if (cmd == "moff") {
    disarmMotors();
  } else if (cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd.length() > 0) {
    Serial2.print("Unknown command: ");
    Serial2.println(cmd);
  }
}

void processChar(char c)
{
  switch (c) {
    case 'w':
    case 'W':
      driveTracks(+1, +1);
      Serial2.println("forward");
      break;

    case 's':
    case 'S':
      driveTracks(-1, -1);
      Serial2.println("reverse");
      break;

    case 'a':
    case 'A':
      driveTracks(-1, +1);
      Serial2.println("left");
      break;

    case 'd':
    case 'D':
      driveTracks(+1, -1);
      Serial2.println("right");
      break;

    case 'x':
    case 'X':
    case ' ':
      stopMotors();
      Serial2.println("stop");
      break;

    case '+':
      drivePower += POWER_STEP;
      if (drivePower > MAX_POWER) drivePower = MAX_POWER;
      Serial2.print("speed = ");
      Serial2.println(drivePower);
      break;

    case '-':
      drivePower -= POWER_STEP;
      if (drivePower < MIN_POWER) drivePower = MIN_POWER;
      Serial2.print("speed = ");
      Serial2.println(drivePower);
      break;
  }
}

void readBluetooth()
{
  while (Serial2.available() > 0) {
    char c = Serial2.read();

    // Show received Bluetooth chars on USB Serial too
    Serial.write(c);

    if (c == '\r' || c == '\n') {
      processLine(lineBuffer);
      lineBuffer = "";
      return;
    }

    if (c == 'w' || c == 'W' ||
        c == 'a' || c == 'A' ||
        c == 's' || c == 'S' ||
        c == 'd' || c == 'D' ||
        c == 'x' || c == 'X' ||
        c == ' ' || c == '+' || c == '-') {
      processChar(c);
      lineBuffer = "";
      return;
    }

    lineBuffer += c;

    if (lineBuffer.length() > 20) {
      lineBuffer = "";
    }
  }
}

void setup_driver()
{
  motorLeft.attach(LEFT_PIN);
  motorRight.attach(RIGHT_PIN);

  stopMotors();

  Serial.begin(115200);   // USB serial to computer
  Serial2.begin(115200);  // Bluetooth on SERIAL2

  delay(500);

  Serial.println("Bluetooth tank drive ready");
  Serial2.println("Bluetooth tank drive ready");
  printHelp();
}

void loop_driver()
{
  readBluetooth();
}