#include "driving_controller.h"

#include <Arduino.h>
#include <math.h>

#include "outputs/DC_motors.h"
#include "inputs/imu.h"
#include "map.h"
#include "pose.h"
#include "state_machine.h"


//to check if tunring or drving straight
enum MotorControlMode
{
    CONTROL_IDLE,
    CONTROL_TURNING,
    CONTROL_DRIVE_HEADING,
    CONTROL_DRIVE_TO_POINT,
    CONTROL_AVOID_TURN
};

//start in idle
static MotorControlMode controlMode = CONTROL_IDLE;

// To be tuned
static float TURN_KP = 22.0;

static float DRIVE_KP = 6.0; // was 10

const int MAX_DRIVE_CORRECTION = 220;

static int driveBasePower = 300;

// Minimum power for robot to actually rotate
const int MIN_TURN_POWER = 280;

const int MAX_TURN_POWER = 450;

static float ARRIVAL_TOLERANCE_MM = 40.0f;
static float SLOWDOWN_RADIUS_MM = 150.0f;
const int MIN_DRIVE_TO_POINT_POWER = 200;


// Consider target reached inside this angle
const float ANGLE_TOLERANCE = 2.5;

const float WALL_AVOID_TRIGGER_MM = 150.0f;


const unsigned long SETTLE_TIME_MS = 100;


// If the robot turns away from the target,
// change this from +1 to -1
const int TURN_SIGN = 1;

const int DRIVE_STEER_SIGN = 1;


static float targetPointX = 0.0f;
static float targetPointY = 0.0f;


static float targetHeading = 0.0;
static float currentError = 0.0;

//varaibles used to chceck how long imu heading has been in tolerance zone
static unsigned long previousTime = 0;
static unsigned long toleranceStart = 0;

const unsigned long CONTROL_PERIOD_MS = 20;





//to make sure the angle is always positive for PD math
static float wrapHeading(float heading)
{
    while (heading >= 360.0) {
        heading -= 360.0;
    }

    while (heading < 0.0) {
        heading += 360.0;
    }

    return heading;
}

void motor_control_set_target_heading(float heading)
{
    targetHeading =
        wrapHeading(heading);
}

//also wraps the heading angle error
static float headingError(float target, float current)
{
    float error = target - current;

    while (error > 180.0) {
        error -= 360.0;
    }

    while (error < -180.0) {
        error += 360.0;
    }

    return error;
}



void motor_control_init()
{
    controlMode = CONTROL_IDLE;

    targetHeading = 0.0;
    currentError = 0.0;

    previousTime = millis();
    toleranceStart = 0;

    DC_motors_setPower(0, 0);
}


//turn to an angle relative to a starting psoition, starting this with bluetooth
void motor_control_turn_relative(float angle)
{
    float currentHeading = imu_get_heading();

    targetHeading = wrapHeading(currentHeading + angle);

    currentError = headingError(targetHeading, currentHeading);


    previousTime = millis();
    toleranceStart = 0;
    controlMode = CONTROL_TURNING;


    // Serial.print("Current heading: ");
    // Serial.println(currentHeading);

    // Serial.print("Relative turn: ");
    // Serial.println(angle);

    // Serial.print("Target heading: ");
    // Serial.println(targetHeading);

    // Serial2.print("Current heading: ");
    // Serial2.println(currentHeading);

    // Serial2.print("Relative turn: ");
    // Serial2.println(angle);

    // Serial2.print("Target heading: ");
    // Serial2.println(targetHeading);
}


void motor_control_turn_to(float heading)
{
    targetHeading = wrapHeading(heading);

    float currentHeading =
        imu_get_heading();

    currentError =
        headingError(
            targetHeading,
            currentHeading
        );

    previousTime = millis();
    toleranceStart = 0;

    controlMode = CONTROL_TURNING;
}

void motor_control_drive_current_heading(int basePower)
{
    targetHeading = imu_get_heading();

    driveBasePower = basePower;

    currentError = 0.0;

    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;   // restore this
}

void motor_control_drive_heading(float heading, int basePower)
{
    targetHeading = wrapHeading(heading);
    driveBasePower = basePower;

    currentError =
        headingError(
            targetHeading,
            imu_get_heading()
        );

    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;   // restore this
}

static void updateTurnControl(float currentHeading, unsigned long currentTime)
{
    currentError =
        headingError(
            targetHeading,
            currentHeading
        );

    //we have reached the imu heading wanted
    if (fabs(currentError) <= ANGLE_TOLERANCE)
    {
        DC_motors_setPower(0, 0);


        if (toleranceStart == 0)
        {
            toleranceStart = currentTime;
        }


        if (
            currentTime - toleranceStart
            >= SETTLE_TIME_MS
        )
        {
            controlMode = CONTROL_IDLE;

            // Serial.print("Turn complete. Heading: ");
            // Serial.println(currentHeading);
        }


        return;
    }


    toleranceStart = 0;


    

    float output =
        TURN_KP * currentError;


    int turnPower =
        abs((int)output);


    if (turnPower < MIN_TURN_POWER)
    {
        turnPower = MIN_TURN_POWER;
    }


    if (turnPower > MAX_TURN_POWER)
    {
        turnPower = MAX_TURN_POWER;
    }


    int direction;

    if (output > 0)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }


    turnPower =
        turnPower
        * direction
        * TURN_SIGN;


    DC_motors_setPower(
        turnPower,
        -turnPower
    );
}




void updateDriveToPointControl(
    float currentHeading,
    float currentX,
    float currentY
)
{
    float dx = targetPointX - currentX;
    float dy = targetPointY - currentY;

    float distance = sqrtf(dx * dx + dy * dy);

    float desiredHeading =
        atan2f(dy, dx) * 180.0f / PI;
    
    // Serial.print("CURRENT ");
    // Serial.print(currentHeading);
    // Serial.print(" DESIRED ");
    // Serial.print(desiredHeading);


    targetHeading = wrapHeading(desiredHeading);

    currentError =
        headingError(targetHeading, currentHeading);

    float correction =
        DRIVE_KP * currentError;

    correction *= DRIVE_STEER_SIGN;

    if (correction > MAX_DRIVE_CORRECTION)
        correction = MAX_DRIVE_CORRECTION;

    if (correction < -MAX_DRIVE_CORRECTION)
        correction = -MAX_DRIVE_CORRECTION;


    int power = driveBasePower;

    if (distance < SLOWDOWN_RADIUS_MM)
    {
        float t = distance / SLOWDOWN_RADIUS_MM; // 0..1
        power = MIN_DRIVE_TO_POINT_POWER +
                (int)((driveBasePower - MIN_DRIVE_TO_POINT_POWER) * t);
    }

    float errorFactor = 1.0f - (fabsf(currentError) / 90.0f);
    if (errorFactor < 0.2f) errorFactor = 0.2f; // never drop below 30% power
    power = (int)(power * errorFactor);

    int leftPower  = power + (int)correction;
    int rightPower = power - (int)correction;

    leftPower  = constrain(leftPower,  0, MAX_TURN_POWER);   // or whatever your true PWM ceiling is
    rightPower = constrain(rightPower, 0, MAX_TURN_POWER);
    
    // Serial.print(" LEFT: ");
    // Serial.print(leftPower);
    // Serial.print(" RIGHT: ");
    // Serial.println(rightPower);


    DC_motors_setPower(
        leftPower,
        rightPower
    );
}


const int AVOID_TURN_STEP_DEG      = 90; 
const int WALL_AVOID_CLEAR_MM      = 250;  
const int AVOID_MAX_ROTATION_DEG   = 350; 

static float avoidStartHeading   = 0.0f;
static float avoidTargetHeading  = 0.0f;
static float avoidTotalRotation  = 0.0f;
static bool  avoidInitialized    = false;
int iteration = 0;
int max_iterations = 10;

// Call this once, right when you switch INTO CONTROL_AVOID_TURN
// (i.e. in motor_control_update(), alongside setting controlMode)
static void initAvoidTurn(float currentHeading)
{
    avoidStartHeading  = currentHeading;
    avoidTargetHeading = wrapHeading(currentHeading + AVOID_TURN_STEP_DEG);
    avoidTotalRotation = 0.0f;
    avoidInitialized   = true;
    toleranceStart      = 0;
    iteration = 0;
}

static void updateAvoidTurnControl(float currentHeading, unsigned long currentTime) {
    if (!avoidInitialized) {
        initAvoidTurn(currentHeading);
    }
    // Serial.println("just checking");

    // Have avoided the wall, yay!
    if (get_front_clearance_mm() > WALL_AVOID_CLEAR_MM)
    {
        // Serial.println("SOMEHOW HERE??!");
        DC_motors_setPower(0, 0);
        avoidInitialized = false;
        controlMode = CONTROL_DRIVE_TO_POINT;
        toleranceStart = 0;
        return;
    }

    // if (headingError(avoidTargetHeading, currentHeading) > ANGLE_TOLERANCE) {
    //     Serial.println("THIS ONE!");
    // }


    if (iteration < max_iterations) {
        
        toleranceStart = 0;

        float output =
            TURN_KP * currentError;

        int turnPower =
            abs((int)output);

        if (turnPower < MIN_TURN_POWER) {
            turnPower = MIN_TURN_POWER;
        }

        if (turnPower > MAX_TURN_POWER) {
            turnPower = MAX_TURN_POWER;
        }

        int direction;

        if (output > 0) {
            direction = 1;
        }
        else {
            direction = -1;
        }


        turnPower =
            turnPower
            * direction
            * TURN_SIGN;


        DC_motors_setPower(
            turnPower,
            -turnPower
        );

        iteration += 1;
        // Serial.println(iteration);

        
    }
    else {
        setStateFlag(&STATE_FLAGS.reverse_triggered);
        controlMode = CONTROL_IDLE;
    }

}





static void updateDriveHeadingControl(
    float currentHeading
)
{
    currentError =headingError(targetHeading, currentHeading);


    float correction = DRIVE_KP * currentError;
    correction *= DRIVE_STEER_SIGN;


    // Don't let heading correction become enormous
    if (correction > MAX_DRIVE_CORRECTION)
    {
        correction = MAX_DRIVE_CORRECTION;
    }

    if (correction < -MAX_DRIVE_CORRECTION)
    {
        correction = -MAX_DRIVE_CORRECTION;
    }


    int leftPower =
        driveBasePower + (int)correction;

    int rightPower =
        driveBasePower - (int)correction;


    DC_motors_setPower(
        leftPower,
        rightPower
    );
}

void motor_control_update()
{
    if (controlMode == CONTROL_IDLE)
    {
        return;
    }


    unsigned long currentTime =
        millis();


    if (
        currentTime - previousTime
        < CONTROL_PERIOD_MS
    )
    {
        return;
    }


    previousTime = currentTime;


    float currentHeading =
        imu_get_heading();
    
    float currentX = pose_get_x_mm();
    float currentY = pose_get_y_mm();


    if (controlMode == CONTROL_TURNING)
    {
        updateTurnControl(
            currentHeading,
            currentTime
        );

        return;
    }


    if (controlMode == CONTROL_DRIVE_HEADING)
    {
        updateDriveHeadingControl(
            currentHeading
        );

        return;
    }

    if (controlMode == CONTROL_DRIVE_TO_POINT)
    {
        if (get_front_clearance_mm() < WALL_AVOID_TRIGGER_MM)
        {
            controlMode = CONTROL_AVOID_TURN;
            initAvoidTurn(currentHeading);
        }
        else
        {
            updateDriveToPointControl(currentHeading, currentX, currentY);
            return;
        }
    }

    if (controlMode == CONTROL_AVOID_TURN)
    {
        updateAvoidTurnControl(currentHeading, currentTime);
        return;
    }
}

void motor_control_drive_to_point(float target_x_mm, float target_y_mm, int basePower)
{
    targetPointX = target_x_mm;
    targetPointY = target_y_mm;
    driveBasePower = basePower;

    if (controlMode != CONTROL_DRIVE_TO_POINT)
    {
        currentError = 0.0f;
        previousTime = millis();
        controlMode = CONTROL_DRIVE_TO_POINT;
    }
}




bool motor_control_is_driving_to_point()
{
    return controlMode == CONTROL_DRIVE_TO_POINT;
}

void motor_control_set_arrival_tolerance(float mm)
{
    ARRIVAL_TOLERANCE_MM = mm;
}

void motor_control_set_slowdown_radius(float mm)
{
    SLOWDOWN_RADIUS_MM = mm;
}

void motor_control_stop()
{
    // Serial.println("stopped");
    controlMode = CONTROL_IDLE;

    DC_motors_setPower(0, 0);
    // Serial.println("Motor control stopped");
    // Serial2.println("Motor control stopped");
}


float motor_control_get_target()
{
    return targetHeading;
}


float motor_control_get_error()
{
    return currentError;
}


void motor_control_set_kp(float kp)
{
    TURN_KP = kp;
}


float motor_control_get_kp()
{
    return TURN_KP;
}




void motor_control_set_drive_kp(float kp)
{
    DRIVE_KP = kp;
}


float motor_control_get_drive_kp()
{
    return DRIVE_KP;
}

bool motor_control_is_turning()
{
    return controlMode == CONTROL_TURNING;
}


bool motor_control_is_driving()
{
    return controlMode == CONTROL_DRIVE_TO_POINT;
}

void motor_control_reverse(int power)
{
    controlMode = CONTROL_IDLE;
    DC_motors_setPower(-power, -power);
}

void print_motor_state () {
    Serial.print("Mode: ");
    Serial.println(controlMode == CONTROL_DRIVE_TO_POINT);
}