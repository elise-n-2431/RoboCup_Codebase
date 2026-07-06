/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position 

void setup() 
{ 
  myservo.attach(1, 500, 2500);  // attaches the servo on pin 9 to the servo object 
  pinMode(26, OUTPUT);
} 
 
void loop() 
{ 
  myservo.write(-10);
  digitalWrite(26, HIGH);
  delay(2000);

    // pos = myservo.read();
    // Serial.print("Servo position 1: ");
    // Serial.println(pos);

  myservo.write(90);
  delay(2000);
  digitalWrite(26, LOW);
  delay(1000);

    // pos = myservo.read(); 
    // Serial.print("Servo position 3: ");
    // Serial.println(pos);
} 
