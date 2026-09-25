#include "weight_detection.h"

#include <Arduino.h>

#include "inputs/tof_expander.h"
#include "debug_print.h"



// Detection tuning


static const int WEIGHT_DETECT_DISTANCE_MM = 650;
static const int WEIGHT_DIFFERENCE_MM = 100;

static const int SIDE_DETECTION_COUNT_REQUIRED = 2;
static const int MIDDLE_DETECTION_COUNT_REQUIRED = 2;

static const unsigned long WEIGHT_EVIDENCE_TIMEOUT_MS = 450;


// Sensor numbers used by tof_get_sample_number()
static const int WEIGHT_LEFT_TOP_SENSOR = 3;
static const int WEIGHT_LEFT_BOTTOM_SENSOR = 4;
static const int WEIGHT_RIGHT_TOP_SENSOR = 2;
static const int WEIGHT_RIGHT_BOTTOM_SENSOR = 1;
static const int WEIGHT_MIDDLE_SENSOR = 8;


// Wall rejection
static const int SIDE_NAV_WALL_TOLERANCE_MM = 120;

static const int INNER_WALL_AGREEMENT_MM = 120;
static const int CENTRE_WALL_OFFSET_MM = 150;
static const int CENTRE_WALL_TOLERANCE_MM = 60;
static const int CENTRE_WEIGHT_PROTRUSION_MM = 60;
static const int CENTRE_ONLY_DETECT_MM = 500;

static const int SIDE_WALL_OFFSET_MM = 150;


// Detection state variables


static int leftDetectionCount = 0;
static int rightDetectionCount = 0;
static int middleDetectionCount = 0;

static unsigned long leftEvidenceAt = 0;
static unsigned long rightEvidenceAt = 0;
static unsigned long middleEvidenceAt = 0;

static unsigned long detectionBlockedUntil = 0;

static uint32_t lastLeftTopSample = 0;
static uint32_t lastLeftBottomSample = 0;

static uint32_t lastRightTopSample = 0;
static uint32_t lastRightBottomSample = 0;

static uint32_t lastMiddleSample = 0;


enum CentreObjectType
{
    CENTRE_UNKNOWN,
    CENTRE_WEIGHT,
    CENTRE_WALL


};

//Checking the wall measruements once a weight detection has been triggered to detremine if it is likely a wall
static bool navSensorSupportsWall(
    int objectDistance,
    int outerLeft,
    int innerLeft,
    int innerRight,
    int outerRight)
{
    int navDistances[4] = {
        outerLeft,
        innerLeft,
        innerRight,
        outerRight
    };

    for (int i = 0; i < 4; i++)
    {
        int navDistance =
            navDistances[i];

        // 0 = no return
        // -1 = invalid
        if (navDistance <= 0)
        {
            continue;
        }

        // Side weight ToFs are farther back than
        // the navigation ToFs, so the same wall
        // appears farther away to the weight sensor.
        int expectedWeightDistance =
            navDistance +
            SIDE_WALL_OFFSET_MM;

        if (abs(
                expectedWeightDistance -
                objectDistance)
            <= SIDE_NAV_WALL_TOLERANCE_MM)
        {
            return true;
        }
    }

    return false;
}

//Function checks if there is a difference between sensors and starts a trigger for a weight being found
static bool weightPairDetected(int top, int bottom, int outerLeft, int innerLeft, int innerRight, int outerRight)
{   
    //if readings are likely invalid dont count a weight
    if (top < 0 || bottom <= 0 || bottom > WEIGHT_DETECT_DISTANCE_MM)
    {
        return false;
    }

    if (top == 0)
    {
        //if top reading is ages away and bottom reading isnt that close then not a weight
        if (bottom > 400) return false;
        //if a wall is close there probably isnt a weight
        if (navSensorSupportsWall(bottom, outerLeft, innerLeft, innerRight, outerRight))
        {
            return false;
        }
        return true;
    }
    //if there is a big enough difference then there is a weight
    return (top - bottom) >= WEIGHT_DIFFERENCE_MM;
}

//Function used for checking triggering weight detection with the centre ToF
static CentreObjectType classifyCentreObject(int middle, int innerLeft, int innerRight)
{
    if (middle <= 0 || middle > WEIGHT_DETECT_DISTANCE_MM)
    {
        return CENTRE_UNKNOWN;
    }

    if (innerLeft < 0 || innerRight < 0)
    {
        return CENTRE_UNKNOWN;
    }

    // Centre sees something while both nav sensors see open space. If close enough call it a weight
    if (innerLeft == 0 && innerRight == 0)
    {
        return middle <= CENTRE_ONLY_DETECT_MM ? CENTRE_WEIGHT : CENTRE_UNKNOWN;
    }

    // One nav sensor sees something and one doesn't.
    if (innerLeft == 0 || innerRight == 0)
    {
        return CENTRE_UNKNOWN;
    }

    int innerAverage = (innerLeft + innerRight) / 2;
    int innerDifference = abs(innerLeft - innerRight);

    //ToF sensor further back then nav sensor so have offset
    int expectedMiddleForWall = innerAverage + CENTRE_WALL_OFFSET_MM;
    int protrusion = expectedMiddleForWall - middle;

    if (innerDifference <= INNER_WALL_AGREEMENT_MM &&
        abs(protrusion) <= CENTRE_WALL_TOLERANCE_MM)
    {
        return CENTRE_WALL;
    }

    if (protrusion >= CENTRE_WEIGHT_PROTRUSION_MM)
    {
        return CENTRE_WEIGHT;
    }

    return CENTRE_UNKNOWN;
}

static void updateLeftEvidence(unsigned long now, int leftTop, int leftBottom, int outerLeft, int innerLeft, int innerRight, int outerRight)
{
    uint32_t topSample = tof_get_sample_number(WEIGHT_LEFT_TOP_SENSOR);
    uint32_t bottomSample = tof_get_sample_number(WEIGHT_LEFT_BOTTOM_SENSOR);

    if (topSample == lastLeftTopSample ||
        bottomSample == lastLeftBottomSample)
    {
        return;
    }

    lastLeftTopSample = topSample;
    lastLeftBottomSample = bottomSample;

    // Invalid reading: retain previous evidence.
    if (leftTop < 0 || leftBottom < 0) return;

    if (weightPairDetected(leftTop, leftBottom, outerLeft, innerLeft, innerRight, outerRight))
    {
        leftDetectionCount++;
        leftEvidenceAt = now;
    }
    else
    {
        leftDetectionCount = 0;
    }
}

static void updateRightEvidence(unsigned long now, int rightTop, int rightBottom, int outerLeft, int innerLeft, int innerRight, int outerRight)
{
    uint32_t topSample = tof_get_sample_number(WEIGHT_RIGHT_TOP_SENSOR);
    uint32_t bottomSample = tof_get_sample_number(WEIGHT_RIGHT_BOTTOM_SENSOR);

    if (topSample == lastRightTopSample ||
        bottomSample == lastRightBottomSample)
    {
        return;
    }

    lastRightTopSample = topSample;
    lastRightBottomSample = bottomSample;

    if (rightTop < 0 || rightBottom < 0) return;

    if (weightPairDetected(rightTop, rightBottom, outerLeft, innerLeft, innerRight, outerRight))
    {
        rightDetectionCount++;
        rightEvidenceAt = now;
    }
    else
    {
        rightDetectionCount = 0;
    }
}

static void updateCentreEvidence(unsigned long now, int middle, int innerLeft, int innerRight)
{
    uint32_t sample = tof_get_sample_number(WEIGHT_MIDDLE_SENSOR);

    if (sample == lastMiddleSample) return;

    lastMiddleSample = sample;

    CentreObjectType type = classifyCentreObject(middle, innerLeft, innerRight);

    // weight in the centre
    if (type == CENTRE_WEIGHT)
    {
        middleDetectionCount++;
        middleEvidenceAt = now;

        debugNav.print("CENTRE WEIGHT EVIDENCE ");
        debugNav.print(middleDetectionCount);
        debugNav.print("/");
        debugNav.println(MIDDLE_DETECTION_COUNT_REQUIRED);
        return;
    }

    if (type == CENTRE_WALL)
    {
        middleDetectionCount = 0;
        return;
    }

    // Object definitely disappeared.
    if (middle == 0 || middle > WEIGHT_DETECT_DISTANCE_MM)
    {
        middleDetectionCount = 0;
        return;
    }

    // Invalid measurements do not erase previous evidence.
    if (middle < 0 || innerLeft < 0 || innerRight < 0)
    {
        return;
    }
    // Valid data, but geometry does not support a weight.
    middleDetectionCount = 0;
}

void weight_detection_reset()
{
    leftDetectionCount = 0;
    rightDetectionCount = 0;
    middleDetectionCount = 0;
}


void weight_detection_reset_side_evidence()
{
    leftDetectionCount = 0;
    rightDetectionCount = 0;
}


void weight_detection_block_for(unsigned long durationMs)
{
    detectionBlockedUntil = millis() + durationMs;
    weight_detection_reset();
}

static void expireOldEvidence(unsigned long now)
{
    if (leftDetectionCount > 0 &&
        now - leftEvidenceAt > WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        leftDetectionCount = 0;
    }

    if (rightDetectionCount > 0 &&
        now - rightEvidenceAt > WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        rightDetectionCount = 0;
    }

    if (middleDetectionCount > 0 &&
        now - middleEvidenceAt > WEIGHT_EVIDENCE_TIMEOUT_MS)
    {
        middleDetectionCount = 0;
    }
}

WeightTargetSide weight_detection_update()
{
    unsigned long now = millis();

    if ((int32_t)(now - detectionBlockedUntil) < 0)
    {
        return TARGET_NONE;
    }

    int leftTop = tof_get_weight_left_top();
    int leftBottom = tof_get_weight_left_bottom();

    int rightTop = tof_get_weight_right_top();
    int rightBottom = tof_get_weight_right_bottom();

    int middle = tof_get_weight_middle();

    int outerLeft = tof_get_nav_outer_left();
    int innerLeft = tof_get_nav_inner_left();
    int innerRight = tof_get_nav_inner_right();
    int outerRight = tof_get_nav_outer_right();

    expireOldEvidence(now);
    updateLeftEvidence(now, leftTop, leftBottom, outerLeft, innerLeft, innerRight, outerRight);
    updateRightEvidence(now, rightTop, rightBottom, outerLeft, innerLeft, innerRight, outerRight);
    updateCentreEvidence(now, middle, innerLeft, innerRight);

    if (middleDetectionCount >= MIDDLE_DETECTION_COUNT_REQUIRED)
    {
        debugNav.println("WEIGHT CANDIDATE CENTRE");
        weight_detection_reset();
        return TARGET_CENTRE;
    }

    if (leftDetectionCount >= SIDE_DETECTION_COUNT_REQUIRED)
    {
        debugNav.println("WEIGHT CANDIDATE LEFT");
        weight_detection_reset();
        return TARGET_LEFT;
    }

    if (rightDetectionCount >= SIDE_DETECTION_COUNT_REQUIRED)
    {
        debugNav.println("WEIGHT CANDIDATE RIGHT");
        weight_detection_reset();
        return TARGET_RIGHT;
    }

    return TARGET_NONE;
}

