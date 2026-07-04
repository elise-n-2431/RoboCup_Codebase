# 1 "C:\\Users\\elise\\AppData\\Local\\Temp\\tmpelyme3pd"
#include <Arduino.h>
# 1 "C:/Users/elise/OneDrive - University of Canterbury/Documents/PlatformIO/Projects/RoboCup_Codebase/src/Robocup.ino"
# 15 "C:/Users/elise/OneDrive - University of Canterbury/Documents/PlatformIO/Projects/RoboCup_Codebase/src/Robocup.ino"
#include <Servo.h>

#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <TaskScheduler.h>


#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "sensors.h"







#define US_READ_TASK_PERIOD 40
#define IR_READ_TASK_PERIOD 40
#define COLOUR_READ_TASK_PERIOD 40
#define SENSOR_AVERAGE_PERIOD 40
#define SET_MOTOR_TASK_PERIOD 40
#define WEIGHT_SCAN_TASK_PERIOD 40
#define COLLECT_WEIGHT_TASK_PERIOD 40
#define RETURN_TO_BASE_TASK_PERIOD 40
#define DETECT_BASE_TASK_PERIOD 40
#define UNLOAD_WEIGHTS_TASK_PERIOD 40
#define CHECK_WATCHDOG_TASK_PERIOD 40
#define VICTORY_DANCE_TASK_PERIOD 40






#define US_READ_TASK_NUM_EXECUTE -1
#define IR_READ_TASK_NUM_EXECUTE -1
#define COLOUR_READ_TASK_NUM_EXECUTE -1
#define SENSOR_AVERAGE_NUM_EXECUTE -1
#define SET_MOTOR_TASK_NUM_EXECUTE -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE -1
#define COLLECT_WEIGHT_TASK_NUM_EXECUTE -1
#define RETURN_TO_BASE_TASK_NUM_EXECUTE -1
#define DETECT_BASE_TASK_NUM_EXECUTE -1
#define UNLOAD_WEIGHTS_TASK_NUM_EXECUTE -1
#define CHECK_WATCHDOG_TASK_NUM_EXECUTE -1
#define VICTORY_DANCE_TASK_NUM_EXECUTE -1


#define IO_POWER 49


#define BAUD_RATE 9600

Servo right_motor;
Servo left_motor;
# 83 "C:/Users/elise/OneDrive - University of Canterbury/Documents/PlatformIO/Projects/RoboCup_Codebase/src/Robocup.ino"
Task tRead_ultrasonic(US_READ_TASK_PERIOD, US_READ_TASK_NUM_EXECUTE, &read_ultrasonic);
Task tRead_infrared(IR_READ_TASK_PERIOD, IR_READ_TASK_NUM_EXECUTE, &read_infrared);
Task tRead_colour(COLOUR_READ_TASK_PERIOD, COLOUR_READ_TASK_NUM_EXECUTE, &read_colour);
Task tSensor_average(SENSOR_AVERAGE_PERIOD, SENSOR_AVERAGE_NUM_EXECUTE, &sensor_average);


Task tSet_motor(SET_MOTOR_TASK_PERIOD, SET_MOTOR_TASK_NUM_EXECUTE, &set_motor);


Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD, WEIGHT_SCAN_TASK_NUM_EXECUTE, &weight_scan);
Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &collect_weight);


Task tReturn_to_base(RETURN_TO_BASE_TASK_PERIOD, RETURN_TO_BASE_TASK_NUM_EXECUTE, &return_to_base);
Task tDetect_base(DETECT_BASE_TASK_PERIOD, DETECT_BASE_TASK_NUM_EXECUTE, &detect_base);
Task tUnload_weights(UNLOAD_WEIGHTS_TASK_PERIOD, UNLOAD_WEIGHTS_TASK_NUM_EXECUTE, &unload_weights);





Scheduler taskManager;




void pin_init();
void robot_init();
void task_init();
void setup();
void loop();
#line 116 "C:/Users/elise/OneDrive - University of Canterbury/Documents/PlatformIO/Projects/RoboCup_Codebase/src/Robocup.ino"
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  robot_init();
  task_init();
  Wire.begin();
  servos_init();
}





void pin_init(){

    Serial.println("Pins have been initialised \n");

    pinMode(IO_POWER, OUTPUT);
    digitalWrite(IO_POWER, 1);
}




void robot_init() {
    Serial.println("Robot is ready \n");
}




void task_init() {


  taskManager.init();


  taskManager.addTask(tRead_ultrasonic);
  taskManager.addTask(tRead_infrared);
  taskManager.addTask(tRead_colour);
  taskManager.addTask(tSensor_average);
  taskManager.addTask(tSet_motor);
  taskManager.addTask(tWeight_scan);
  taskManager.addTask(tCollect_weight);
  taskManager.addTask(tReturn_to_base);
  taskManager.addTask(tDetect_base);
  taskManager.addTask(tUnload_weights);





  tRead_ultrasonic.enable();
  tRead_infrared.enable();
  tRead_colour.enable();
  tSensor_average.enable();
  tSet_motor.enable();
  tWeight_scan.enable();
  tCollect_weight.enable();
  tReturn_to_base.enable();
  tDetect_base.enable();
  tUnload_weights.enable();



 Serial.println("Tasks have been initialised \n");
}






void loop() {

  taskManager.execute();
  Serial.println("Another scheduler execution cycle has oocured \n");
}