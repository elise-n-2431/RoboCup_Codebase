#include "priority_targets.h"

#include <math.h>

#include "arena_config.h"
#include "competition_setup.h"

static PriorityTarget targets[
    MAX_PRIORITY_TARGETS
];

static uint8_t targetCount = 0;


void priority_targets_clear()
{
    targetCount = 0;
}

void priority_targets_load_starting_weights()
{
    priority_targets_clear();

    for (int i = 0;
         i < NUM_STARTING_WEIGHTS;
         i++)
    {
        priority_targets_add(
            STARTING_WEIGHTS[i].x,
            STARTING_WEIGHTS[i].y
        );
    }
}


bool priority_targets_add(
    float x,
    float y
)
{
    if (!isfinite(x) ||
        !isfinite(y))
    {
        return false;
    }


    if (x < 0.0f ||
        x > ARENA_X_MM ||
        y < 0.0f ||
        y > ARENA_Y_MM)
    {
        return false;
    }


    if (targetCount >=
        MAX_PRIORITY_TARGETS)
    {
        return false;
    }


    targets[targetCount].x = x;
    targets[targetCount].y = y;

    targetCount++;

    return true;
}


bool priority_targets_remove(
    uint8_t index
)
{
    if (index >= targetCount)
    {
        return false;
    }


    for (uint8_t i = index;
         i + 1 < targetCount;
         i++)
    {
        targets[i] =
            targets[i + 1];
    }


    targetCount--;

    return true;
}


uint8_t priority_targets_count()
{
    return targetCount;
}


bool priority_targets_get(
    uint8_t index,
    float &x,
    float &y
)
{
    if (index >= targetCount)
    {
        return false;
    }


    x = targets[index].x;
    y = targets[index].y;

    return true;
}


void priority_targets_print(
    Stream &port
)
{
    port.print("TARGETS,");
    port.println(targetCount);


    for (uint8_t i = 0;
         i < targetCount;
         i++)
    {
        port.print("TARGET,");
        port.print(i);

        port.print(",");
        port.print(
            targets[i].x,
            0
        );

        port.print(",");
        port.println(
            targets[i].y,
            0
        );
    }
}

bool priority_targets_remove_nearest(
    float x,
    float y,
    float maxDistanceMm
)
{
    if (targetCount == 0 ||
        !isfinite(x) ||
        !isfinite(y) ||
        !isfinite(maxDistanceMm) ||
        maxDistanceMm <= 0.0f)
    {
        return false;
    }


    int bestIndex = -1;

    float bestDistanceSquared =
        maxDistanceMm *
        maxDistanceMm;


    for (uint8_t i = 0;
         i < targetCount;
         i++)
    {
        float dx =
            targets[i].x -
            x;

        float dy =
            targets[i].y -
            y;


        float distanceSquared =
            dx * dx +
            dy * dy;


        if (distanceSquared <=
            bestDistanceSquared)
        {
            bestDistanceSquared =
                distanceSquared;

            bestIndex =
                static_cast<int>(i);
        }
    }


    if (bestIndex < 0)
    {
        return false;
    }


    return priority_targets_remove(
        static_cast<uint8_t>(
            bestIndex
        )
    );
}