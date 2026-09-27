#include "rejected_weights.h"

#include <Arduino.h>
#include <math.h>

#include "pose.h"
#include "debug_print.h"


struct RejectedWeight
{
    float x;
    float y;
};


static const int MAX_REJECTED_WEIGHTS = 8;

static const float REJECT_RADIUS_MM = 100.0f;
static const float DUPLICATE_RADIUS_MM = 250.0f;

static RejectedWeight rejectedWeights[
    MAX_REJECTED_WEIGHTS
];

static int rejectedWeightCount = 0;


static float distanceBetween(
    float x1,
    float y1,
    float x2,
    float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;

    return sqrtf(
        dx * dx +
        dy * dy
    );
}


void rejected_weights_reset()
{
    rejectedWeightCount = 0;
}


void rejected_weights_add_current()
{
    float x = pose_get_x_mm();
    float y = pose_get_y_mm();

    // Don't save the same dummy repeatedly.
    for (int i = 0;
         i < rejectedWeightCount;
         i++)
    {
        float distance =
            distanceBetween(
                x,
                y,
                rejectedWeights[i].x,
                rejectedWeights[i].y
            );

        if (distance <=
            DUPLICATE_RADIUS_MM)
        {
            return;
        }
    }

    if (rejectedWeightCount >=
        MAX_REJECTED_WEIGHTS)
    {
        return;
    }

    rejectedWeights[
        rejectedWeightCount
    ].x = x;

    rejectedWeights[
        rejectedWeightCount
    ].y = y;

    debugNav.print("NAV_EVENT,");
    debugNav.print(millis());
    debugNav.print(",DUMMY_STORED,");
    debugNav.print(
        rejectedWeightCount
    );
    debugNav.print(",");
    debugNav.print(x);
    debugNav.print(",");
    debugNav.println(y);

    rejectedWeightCount++;
}


bool rejected_weights_is_near(
    float x_mm,
    float y_mm,
    float &distance_mm)
{
    distance_mm = -1.0f;

    for (int i = 0;
         i < rejectedWeightCount;
         i++)
    {
        float distance =
            distanceBetween(
                x_mm,
                y_mm,
                rejectedWeights[i].x,
                rejectedWeights[i].y
            );

        if (distance <=
            REJECT_RADIUS_MM)
        {
            distance_mm = distance;
            return true;
        }
    }

    return false;
}


int rejected_weights_count()
{
    return rejectedWeightCount;
}