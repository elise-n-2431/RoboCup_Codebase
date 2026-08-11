#include "navigator.h"
#include "map.h"
#include <Arduino.h>
#include <math.h>

static NavCommand currentCommand = {0, 0};


/* Note: just placeholder, not written by me */
/* Takes map and decides motor motion */


// How far off-heading (in "cell direction" terms) we tolerate before
// treating ourselves as aligned and driving straight instead of turning.
const int ALIGN_TOLERANCE = 0;

void navigator_init()
{
    currentCommand = {0, 0};
}

// Very simple frontier-seeking: scan the map for the nearest cell marked
// FRONTIER, then point roughly at it. This is a starting structure, not
// real path planning — no obstacle avoidance, no shortest-path routing.
static bool findNearestFrontier(int &targetX, int &targetY)
{
    int bestDistSq = -1;

    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            if (map[x][y] != 3 /* frontier */) {
                continue;
            }

            int dx = x - self_x;
            int dy = y - self_y;
            int distSq = dx * dx + dy * dy;

            if (bestDistSq < 0 || distSq < bestDistSq) {
                bestDistSq = distSq;
                targetX = x;
                targetY = y;
            }
        }
    }

    return bestDistSq >= 0;
}

// Turns coarse dx/dy toward a target into a track command.
// TODO: replace with heading-based steering once heading_rad is reliable —
// this version only looks at which axis has the bigger gap, it doesn't
// know which way the robot is actually facing.
static NavCommand commandTowards(int targetX, int targetY)
{
    int dx = targetX - self_x;
    int dy = targetY - self_y;

    if (dx == 0 && dy == 0) {
        return {0, 0}; // arrived
    }

    if (abs(dx) > abs(dy)) {
        return (dx > 0) ? NavCommand{+1, -1} : NavCommand{-1, +1}; // turn toward +x / -x
    }

    return (dy > 0) ? NavCommand{+1, +1} : NavCommand{-1, -1}; // drive toward +y / -y
}

void navigator_exe()
{
    int targetX, targetY;

    if (!findNearestFrontier(targetX, targetY)) {
        currentCommand = {0, 0}; // nothing left to explore — hold position
        return;
    }

    currentCommand = commandTowards(targetX, targetY);
}

NavCommand navigator_getCommand()
{
    return currentCommand;
}