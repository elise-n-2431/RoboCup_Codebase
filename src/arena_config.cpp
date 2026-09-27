#include "arena_config.h"
#include <math.h>
#include "competition_setup.h"

// Existing position/heading defaults. Colour labels the sampled home base;
// it does not substitute uncalibrated RGB thresholds for the colour sensor.
static ArenaConfig config = {
    HomeColour::BLUE,

    COMPETITION_HOME_X,
    COMPETITION_HOME_Y,

    COMPETITION_START_X,
    COMPETITION_START_Y,
    COMPETITION_START_HEADING,

    true
};

static bool locked = false;
static bool started = false;

static bool validPosition(float x, float y)
{
    return isfinite(x) && isfinite(y) &&
           x >= 0.0f && x <= ARENA_X_MM &&
           y >= 0.0f && y <= ARENA_Y_MM;
}

const ArenaConfig& arena_get_config() { return config; }
float arena_get_home_x_mm() { return config.homeX; }
float arena_get_home_y_mm() { return config.homeY; }
bool arena_is_locked() { return locked; }
bool arena_run_started() { return started; }
void arena_lock() { locked = true; }
void arena_begin_run() { locked = true; started = true; }

bool arena_set_home_colour(HomeColour colour)
{
    if (locked ||
        (colour != HomeColour::BLUE && colour != HomeColour::GREEN))
        return false;

    config.colour = colour;
    return true;
}

bool arena_set_home_position(float x, float y)
{
    if (locked || !validPosition(x, y)) return false;

    config.homeX = x;
    config.homeY = y;
    return true;
}

bool arena_set_home_corner(HomeCorner corner)
{
    const float near = BASE_SIZE_MM / 2.0f;

    switch (corner) {
        case HomeCorner::SW:
            return arena_set_home_position(near, near);

        case HomeCorner::SE:
            return arena_set_home_position(ARENA_X_MM - near, near);

        case HomeCorner::NW:
            return arena_set_home_position(near, ARENA_Y_MM - near);

        case HomeCorner::NE:
            return arena_set_home_position(
                ARENA_X_MM - near, ARENA_Y_MM - near);
    }

    return false;
}

bool arena_set_start_pose(float x, float y, float heading)
{
    if (locked || !validPosition(x, y) || !isfinite(heading))
        return false;

    heading = fmodf(heading, 360.0f);
    if (heading < 0.0f) heading += 360.0f;

    config.startX = x;
    config.startY = y;
    config.startHeading = heading;
    return true;
}

bool arena_set_pickup_enabled(bool enabled)
{
    if (locked) return false;

    config.pickupEnabled = enabled;
    return true;
}

void arena_sync_start_to_home()
{
    config.startX = config.homeX;
    config.startY = config.homeY;
}