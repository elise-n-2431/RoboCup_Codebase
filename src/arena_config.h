#ifndef ARENA_CONFIG_H
#define ARENA_CONFIG_H

enum class HomeColour { BLUE, GREEN };
enum class HomeCorner { SW, SE, NW, NE };

// Arena coordinates: +X along the 4.9 m side, +Y along the 2.4 m side.
// Heading 0 faces +X; 90 faces +Y, preserving the existing pose convention.
constexpr float ARENA_X_MM = 4900.0f;
constexpr float ARENA_Y_MM = 2400.0f;
constexpr float BASE_SIZE_MM = 600.0f;

struct ArenaConfig {
    HomeColour colour;
    float homeX, homeY;
    float startX, startY, startHeading;
    bool pickupEnabled;
};

const ArenaConfig& arena_get_config();
bool arena_set_home_colour(HomeColour colour);
bool arena_set_home_corner(HomeCorner corner);
bool arena_set_home_position(float x, float y);
bool arena_set_start_pose(float x, float y, float heading);
bool arena_set_pickup_enabled(bool enabled);
float arena_get_home_x_mm();
float arena_get_home_y_mm();
void arena_lock();
bool arena_is_locked();
void arena_begin_run();
bool arena_run_started();

#endif