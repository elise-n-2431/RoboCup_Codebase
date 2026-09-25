#include "driving_controller.h"

#include <Arduino.h>
#include <math.h>

#include "outputs/DC_motors.h"
#include "inputs/imu.h"
#include "inputs/tof_expander.h"
#include "state_machine.h"
#include "debug_print.h"
#include "pose.h"


// ============================================================
// CONTROL MODES
// ============================================================

enum MotorControlMode
{
    CONTROL_IDLE,
    CONTROL_TURNING,
    CONTROL_DRIVE_HEADING,
    CONTROL_DRIVE_TO_POINT,
    CONTROL_AVOID_TURN
};

static MotorControlMode controlMode = CONTROL_IDLE;


// ============================================================
// GENERAL CONTROL TUNING
// ============================================================

static float TURN_KP = 16.0f;
static float DRIVE_KP = 5.0f;


static const int MAX_DRIVE_CORRECTION = 100;
static const int MAX_POINT_CORRECTION = 260;

static int driveBasePower = 380;

static const int MIN_TURN_POWER = 270;
static const int MAX_MOTOR_POWER = 450;

static const float ANGLE_TOLERANCE_DEG = 3.0f;
static const unsigned long SETTLE_TIME_MS = 100;

static const int TURN_SIGN = 1;
static const int DRIVE_STEER_SIGN = 1;
static const int POINT_STEER_SIGN = -1;

static const unsigned long CONTROL_PERIOD_MS = 20;


static const float FINE_TURN_ZONE_DEG = 15.0f;

static const unsigned long FINE_TURN_ON_MS = 50;
static const unsigned long FINE_TURN_OFF_MS = 70;

static unsigned long fineTurnPhaseStarted = 0;
static bool fineTurnPowerOn = true;


static float ARRIVAL_TOLERANCE_MM = 40.0f;
static float SLOWDOWN_RADIUS_MM = 150.0f;

static const int MIN_DRIVE_TO_POINT_POWER = 260;


// Enter avoidance when something gets this close.
static const int WALL_AVOID_TRIGGER_MM = 120;

// Do not return to the target until we have clearly opened
// up some space again.
static const int WALL_AVOID_CLEAR_MM = 250;

// Safety limits. If local avoidance cannot solve the problem,
// hand over to the reversing controller.
static const float AVOID_MAX_ROTATION_DEG = 350.0f;
static const unsigned long AVOID_MAX_TIME_MS = 2000;

static bool avoidInitialized = false;

// -1 = LEFT
// +1 = RIGHT
static int avoidDirection = 1;
static int fallbackAvoidDirection = 1;

static float avoidLastHeading = 0.0f;
static float avoidTotalRotation = 0.0f;

static unsigned long avoidStartedAt = 0;


static float targetHeading = 0.0f;
static float currentError = 0.0f;

static unsigned long previousTime = 0;
static unsigned long toleranceStart = 0;


static float targetPointX = 0.0f;
static float targetPointY = 0.0f;

static bool pointTargetValid = false;
static bool pointReached = false;


//for debugging
static int lastLeftPower = 0;
static int lastRightPower = 0;

static float lastPointDistance = -1.0f;

static unsigned long lastMotorTelemetryAt = 0;
static const unsigned long MOTOR_TELEMETRY_PERIOD_MS = 150;

static const char* motorModeName()
{
    switch (controlMode)
    {
        case CONTROL_IDLE: return "IDLE";
        case CONTROL_TURNING: return "TURNING";
        case CONTROL_DRIVE_HEADING: return "DRIVE_HEADING";
        case CONTROL_DRIVE_TO_POINT: return "DRIVE_POINT";
        case CONTROL_AVOID_TURN: return "AVOID";
        default: return "UNKNOWN";
    }
}

static void setMotorPower(
    int leftPower,
    int rightPower)
{
    lastLeftPower = leftPower;
    lastRightPower = rightPower;

    DC_motors_setPower(
        leftPower,
        rightPower
    );
}


static float wrapHeading(float heading)
{
    while (heading >= 360.0f) heading -= 360.0f;
    while (heading < 0.0f) heading += 360.0f;

    return heading;
}


static float headingError(float target, float current)
{
    float error = target - current;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}


static int clearanceValue(int distance)
{
    // 0 = no obstacle return.
    // -1 = invalid/unavailable.
    // For local direction choice, treat either as open space.
    if (distance <= 0)
    {
        return 1200;
    }

    return distance;
}


static int getFrontClearance()
{
    int outerLeft = clearanceValue(tof_get_nav_outer_left());
    int innerLeft = clearanceValue(tof_get_nav_inner_left());
    int innerRight = clearanceValue(tof_get_nav_inner_right());
    int outerRight = clearanceValue(tof_get_nav_outer_right());

    return min(
        min(outerLeft, innerLeft),
        min(innerRight, outerRight)
    );
}


static int getLeftClearance()
{
    return min(
        clearanceValue(tof_get_nav_outer_left()),
        clearanceValue(tof_get_nav_inner_left())
    );
}


static int getRightClearance()
{
    return min(
        clearanceValue(tof_get_nav_inner_right()),
        clearanceValue(tof_get_nav_outer_right())
    );
}


static void clearPointTarget()
{
    pointTargetValid = false;
    pointReached = false;

    avoidInitialized = false;
    avoidTotalRotation = 0.0f;
    lastPointDistance = -1.0f;
}



void motor_control_init()
{
    controlMode = CONTROL_IDLE;

    targetHeading = 0.0f;
    currentError = 0.0f;

    previousTime = millis();
    toleranceStart = 0;

    fineTurnPhaseStarted = 0;
    fineTurnPowerOn = true;

    clearPointTarget();

    setMotorPower(0, 0);
}


void motor_control_turn_relative(float angle)
{
    clearPointTarget();

    float currentHeading = imu_get_heading();

    targetHeading = wrapHeading(currentHeading + angle);
    currentError = headingError(targetHeading, currentHeading);

    previousTime = millis();
    toleranceStart = 0;

    controlMode = CONTROL_TURNING;

    debugMotor.print("Current heading: ");
    debugMotor.println(currentHeading);

    debugMotor.print("Relative turn: ");
    debugMotor.println(angle);

    debugMotor.print("Target heading: ");
    debugMotor.println(targetHeading);
}


void motor_control_turn_to(float heading)
{
    clearPointTarget();

    float currentHeading = imu_get_heading();

    targetHeading = wrapHeading(heading);
    currentError = headingError(targetHeading, currentHeading);

    previousTime = millis();
    toleranceStart = 0;

    controlMode = CONTROL_TURNING;
}


void motor_control_drive_current_heading(int basePower)
{
    clearPointTarget();

    targetHeading = imu_get_heading();
    driveBasePower = basePower;

    currentError = 0.0f;
    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;

    debugMotor.print("Driving at heading: ");
    debugMotor.println(targetHeading);

    debugMotor.print("Base power: ");
    debugMotor.println(driveBasePower);
}


void motor_control_drive_heading(float heading, int basePower)
{
    clearPointTarget();

    targetHeading = wrapHeading(heading);
    driveBasePower = basePower;

    currentError = headingError(
        targetHeading,
        imu_get_heading()
    );

    previousTime = millis();

    controlMode = CONTROL_DRIVE_HEADING;
}

static void updateTurnControl(
    float currentHeading,
    unsigned long currentTime)
{
    currentError = headingError(
        targetHeading,
        currentHeading
    );

    float absError = fabsf(currentError);

    // Target reached.
    if (absError <= ANGLE_TOLERANCE_DEG)
    {
        setMotorPower(0, 0);

        fineTurnPhaseStarted = 0;
        fineTurnPowerOn = true;

        if (toleranceStart == 0)
        {
            toleranceStart = currentTime;
        }

        if (currentTime - toleranceStart >= SETTLE_TIME_MS)
        {
            controlMode = CONTROL_IDLE;

            debugMotor.print("Turn complete. Heading: ");
            debugMotor.println(currentHeading);
        }

        return;
    }

    toleranceStart = 0;

    float output = TURN_KP * currentError;

    int direction =
        output > 0.0f
        ? 1
        : -1;

    // Fine turning near target.
    if (absError <= FINE_TURN_ZONE_DEG)
    {
        if (fineTurnPhaseStarted == 0)
        {
            fineTurnPhaseStarted = currentTime;
            fineTurnPowerOn = true;
        }

        unsigned long phaseTime =
            currentTime - fineTurnPhaseStarted;

        if (fineTurnPowerOn)
        {
            if (phaseTime >= FINE_TURN_ON_MS)
            {
                fineTurnPowerOn = false;
                fineTurnPhaseStarted = currentTime;
            }
        }
        else
        {
            if (phaseTime >= FINE_TURN_OFF_MS)
            {
                fineTurnPowerOn = true;
                fineTurnPhaseStarted = currentTime;
            }
        }

        if (!fineTurnPowerOn)
        {
            setMotorPower(0, 0);
            return;
        }

        int turnPower =
            MIN_TURN_POWER *
            direction *
            TURN_SIGN;

        setMotorPower(
            turnPower,
            -turnPower
        );

        return;
    }

    // Normal larger turn.
    fineTurnPhaseStarted = 0;
    fineTurnPowerOn = true;

    int turnPower = abs((int)output);

    turnPower = constrain(
        turnPower,
        MIN_TURN_POWER,
        MAX_MOTOR_POWER
    );

    turnPower *= direction * TURN_SIGN;

    setMotorPower(
        turnPower,
        -turnPower
    );
}

static void updateDriveHeadingControl(float currentHeading)
{
    currentError = headingError(
        targetHeading,
        currentHeading
    );

    float correction =
        DRIVE_KP * currentError * DRIVE_STEER_SIGN;

    correction = constrain(
        correction,
        -MAX_DRIVE_CORRECTION,
        MAX_DRIVE_CORRECTION
    );

    int leftPower =
        driveBasePower +
        (int)correction;

    int rightPower =
        driveBasePower -
        (int)correction;

    setMotorPower(
        leftPower,
        rightPower
    );
}


static void updateDriveToPointControl(
    float currentPoseHeading,
    float currentX,
    float currentY)
{
    float dx = targetPointX - currentX;
    float dy = targetPointY - currentY;

    float distance =
        sqrtf(dx * dx + dy * dy);

    lastPointDistance = distance;
    // We have reached this XY target.
    if (distance <= ARRIVAL_TOLERANCE_MM)
    {
        setMotorPower(0, 0);

        pointReached = true;
        controlMode = CONTROL_IDLE;

        debugMotor.print("Drive-to-point reached target, distance=");
        debugMotor.println(distance);

        return;
    }

    // atan2 gives an absolute heading in the POSE coordinate frame.
    float desiredHeading =
        atan2f(dy, dx) *
        180.0f / PI;

    targetHeading =
        wrapHeading(desiredHeading);

    currentError = headingError(
        targetHeading,
        currentPoseHeading
    );
    float steer =
        DRIVE_KP *
        currentError *
        POINT_STEER_SIGN;

    // Normal forward speed.
    int power = driveBasePower;

    // Only slow because we are physically near the target.
    if (distance < SLOWDOWN_RADIUS_MM)
    {
        float t =
            distance /
            SLOWDOWN_RADIUS_MM;

        power =
            MIN_DRIVE_TO_POINT_POWER +
            (int)(
                (driveBasePower -
                MIN_DRIVE_TO_POINT_POWER) *
                t
            );
    }

    // Never stop the inside track during ordinary point following.
    // This is especially important when crossing the home-base lip.
    static const int MIN_CURVE_POWER = 200;

    // Convert steering error into a left-right speed difference.
    // Factor 2 preserves roughly the same turning authority as the
    // old +/- correction arrangement.
    int speedDifference =
        abs((int)(2.0f * steer));

    speedDifference = constrain(
        speedDifference,
        0,
        power - MIN_CURVE_POWER
    );

    int leftPower = power;
    int rightPower = power;

    if (steer > 0.0f)
    {
        // Turn by slowing right track.
        rightPower =
            power - speedDifference;
    }
    else if (steer < 0.0f)
    {
        // Turn by slowing left track.
        leftPower =
            power - speedDifference;
    }

    setMotorPower(
        leftPower,
        rightPower
    );
}


static void initAvoidTurn(float currentHeading)
{
    int leftClearance = getLeftClearance();
    int rightClearance = getRightClearance();

    if (abs(leftClearance - rightClearance) > 80)
    {
        avoidDirection =
            leftClearance > rightClearance
            ? -1
            : 1;
    }
    else
    {
        avoidDirection = fallbackAvoidDirection;
        fallbackAvoidDirection *= -1;
    }

    avoidLastHeading = currentHeading;
    avoidTotalRotation = 0.0f;
    avoidStartedAt = millis();
    avoidInitialized = true;

    debugMotor.print("Point avoidance: ");
    debugMotor.println(
        avoidDirection < 0
        ? "LEFT"
        : "RIGHT"
    );
}


static void updateAvoidTurnControl(
    float currentHeading,
    unsigned long currentTime)
{
    if (!avoidInitialized)
    {
        initAvoidTurn(currentHeading);
    }

    // Track how much we have rotated while attempting to get clear.
    float rotationStep =
        fabsf(
            headingError(
                currentHeading,
                avoidLastHeading
            )
        );

    avoidTotalRotation += rotationStep;
    avoidLastHeading = currentHeading;

    // Once clearly away from the obstacle, resume the same XY target.
    if (getFrontClearance() > WALL_AVOID_CLEAR_MM)
    {
        avoidInitialized = false;
        avoidTotalRotation = 0.0f;

        controlMode = CONTROL_DRIVE_TO_POINT;

        debugMotor.println(
            "Point avoidance clear - resuming target"
        );

        return;
    }

    // If local avoidance cannot solve it, let the reversing
    // controller perform the more aggressive recovery.
    if (avoidTotalRotation >= AVOID_MAX_ROTATION_DEG ||
        currentTime - avoidStartedAt >= AVOID_MAX_TIME_MS)
    {
        setMotorPower(0, 0);

        avoidInitialized = false;
        avoidTotalRotation = 0.0f;

        controlMode = CONTROL_IDLE;

        debugMotor.println(
            "Point avoidance failed - triggering reverse"
        );

        setStateFlag(
            &STATE_FLAGS.reverse_triggered
        );

        return;
    }

    // Arc forward around the obstacle instead of point-turning.
    // Both tracks remain forward, but the inside track is slower.
    int outerPower = constrain(
        driveBasePower,
        MIN_DRIVE_TO_POINT_POWER,
        MAX_MOTOR_POWER
    );

    int innerPower =
        MIN_DRIVE_TO_POINT_POWER;

    if (avoidDirection < 0)
    {
        // LEFT arc.
        setMotorPower(
            innerPower,
            outerPower
        );
    }
    else
    {
        // RIGHT arc.
        setMotorPower(
            outerPower,
            innerPower
        );
    }
}



static void printMotorTelemetry()
{
    if (!debugMotor.enabled) return;

    if (millis() - lastMotorTelemetryAt <
        MOTOR_TELEMETRY_PERIOD_MS)
    {
        return;
    }

    lastMotorTelemetryAt = millis();

    debugMotor.print("MOTOR,");
    debugMotor.print(millis());

    debugMotor.print(",");
    debugMotor.print(motorModeName());

    debugMotor.print(",");
    debugMotor.print(pose_get_x_mm());

    debugMotor.print(",");
    debugMotor.print(pose_get_y_mm());

    debugMotor.print(",");
    debugMotor.print(pose_get_heading_deg());

    debugMotor.print(",");
    debugMotor.print(targetHeading);

    debugMotor.print(",");
    debugMotor.print(currentError);

    debugMotor.print(",");
    debugMotor.print(targetPointX);

    debugMotor.print(",");
    debugMotor.print(targetPointY);

    debugMotor.print(",");
    debugMotor.print(lastPointDistance);

    debugMotor.print(",");
    debugMotor.print(getFrontClearance());

    debugMotor.print(",");
    debugMotor.print(lastLeftPower);

    debugMotor.print(",");
    debugMotor.println(lastRightPower);
}


void motor_control_update()
{
    if (controlMode == CONTROL_IDLE)
    {
        return;
    }

    unsigned long currentTime = millis();

    if (currentTime - previousTime <
        CONTROL_PERIOD_MS)
    {
        return;
    }

    previousTime = currentTime;

    float currentImuHeading =
        imu_get_heading();

    if (controlMode == CONTROL_TURNING)
    {
        updateTurnControl(
            currentImuHeading,
            currentTime
        );
    }
    else if (controlMode == CONTROL_DRIVE_HEADING)
    {
        updateDriveHeadingControl(
            currentImuHeading
        );
    }
    else if (controlMode == CONTROL_DRIVE_TO_POINT)
    {
        if (getFrontClearance() <
            WALL_AVOID_TRIGGER_MM)
        {
            controlMode =
                CONTROL_AVOID_TURN;

            initAvoidTurn(
                currentImuHeading
            );
        }
        else
        {
            updateDriveToPointControl(
                pose_get_heading_deg(),
                pose_get_x_mm(),
                pose_get_y_mm()
            );
        }
    }
    else if (controlMode == CONTROL_AVOID_TURN)
    {
        updateAvoidTurnControl(
            currentImuHeading,
            currentTime
        );
    }

    printMotorTelemetry();
}


void motor_control_drive_to_point(
    float target_x_mm,
    float target_y_mm,
    int basePower)
{
    bool newTarget =
        !pointTargetValid ||
        fabsf(target_x_mm - targetPointX) > 1.0f ||
        fabsf(target_y_mm - targetPointY) > 1.0f;

    targetPointX = target_x_mm;
    targetPointY = target_y_mm;
    driveBasePower = basePower;

    if (newTarget)
    {
        //first tell the gui
        debugMotor.print("MOTOR_EVENT,");
        debugMotor.print(millis());
        debugMotor.print(",POINT_START,");
        debugMotor.print(targetPointX);
        debugMotor.print(",");
        debugMotor.println(targetPointY);
        pointTargetValid = true;
        pointReached = false;

        debugMotor.print("New point target: ");
        debugMotor.print(targetPointX);
        debugMotor.print(",");
        debugMotor.println(targetPointY);
    }

    // Navigator may repeatedly request the same target.
    // Don't restart once it has already been reached.
    if (pointReached)
    {
        return;
    }

    // Do not interrupt local avoidance for repeated commands
    // to the same target.
    if (controlMode != CONTROL_DRIVE_TO_POINT &&
        controlMode != CONTROL_AVOID_TURN)
    {
        currentError = 0.0f;
        previousTime = millis();

        controlMode = CONTROL_DRIVE_TO_POINT;
    }
}


void motor_control_stop()
{
    controlMode = CONTROL_IDLE;

    clearPointTarget();

    setMotorPower(0, 0);

    debugMotor.println(
        "Motor control stopped"
    );
}


void motor_control_reverse(int power)
{
    controlMode = CONTROL_IDLE;

    clearPointTarget();

    setMotorPower(
        -power,
        -power
    );
}



bool motor_control_is_turning()
{
    return controlMode ==
           CONTROL_TURNING;
}


bool motor_control_is_driving()
{
    // Preserve the meaning used by pursuit/homing.
    return controlMode ==
           CONTROL_DRIVE_HEADING;
}


bool motor_control_is_driving_to_point()
{
    return controlMode == CONTROL_DRIVE_TO_POINT ||
           controlMode == CONTROL_AVOID_TURN;
}


bool motor_control_point_reached()
{
    return pointTargetValid &&
           pointReached;
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


void motor_control_set_arrival_tolerance(float mm)
{
    if (mm > 0.0f)
    {
        ARRIVAL_TOLERANCE_MM = mm;
    }
}


void motor_control_set_slowdown_radius(float mm)
{
    if (mm > ARRIVAL_TOLERANCE_MM)
    {
        SLOWDOWN_RADIUS_MM = mm;
    }
}
