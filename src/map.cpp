#include <stdint.h>
#include <HardwareSerial.h>
#include "pose.h"
#include <list>
#include <cstdint>
#include <cmath>
#include "inputs/tof_expander.h"

const int CELL_SIZE_MM = 50;

const int MAP_WIDTH = 40;
const int MAP_HEIGHT = 40;

// Navigation sensors
const int NAV_OUTER_LEFT  = 6;
const int NAV_INNER_LEFT  = 5;
const int NAV_INNER_RIGHT = 0;
const int NAV_OUTER_RIGHT = 7;

// Weight detection sensors
const int WEIGHT_LEFT_TOP     = 4;
const int WEIGHT_LEFT_BOTTOM  = 3;
const int WEIGHT_RIGHT_TOP    = 2;
const int WEIGHT_RIGHT_BOTTOM = 1;
const int WEIGHT_MIDDLE = 8;

int max_iter = 20;
int iteration = 0;
// --- Fixed-point confidence values ---------------------------------------
// Both maps store confidence as int16_t / uint16_t scaled by CONF_SCALE,
// i.e. "1000" means 1.000, "250" means 0.250, etc. -- 3 decimal places of
// precision, 2 bytes/cell, and every operation below is plain integer
// arithmetic (no float ops, no FPU needed).
//
// int16_t range is -32768..32767, so +-1000 has huge headroom -- no
// overflow risk even after repeated multiply/divide in decay.

const int16_t CONF_SCALE = 1000;                // 1.000 in fixed-point units
const int16_t OBSTACLE_UNKNOWN_BAND = 100;      // |value| below this counts as "unknown" (0.100)

uint16_t WEIGHT_MAP[MAP_WIDTH][MAP_HEIGHT]; 
int16_t  OBSTACLE_MAP[MAP_WIDTH][MAP_HEIGHT];   // +-32768, but we only use +-1000

int self_x = 0; // define on startup
int self_y = 0;

float MAP_ORIGIN_X_MM = 0;
float MAP_ORIGIN_Y_MM = 0;

int home_x = 0;
int home_y = 0;

const int n = 3; // number of starting weight estimates
int starting_weight_estimates[n][2] = {{4, 5}, {7, 9}, {50, 40}};

// Decay factors as integer "permille" (parts per thousand), e.g. 998 == 0.998.
// Applied as: new_val = (val * PERMILLE) / 1000, all in integer math.
const int32_t DECAY_WEIGHT_PERMILLE   = 995;
const int32_t DECAY_OBSTACLE_PERMILLE = 997; 
const int32_t DECAY_FREE_PERMILLE     = 997; 

const int32_t ARENA_MIRROR_PERMILLE = 400; 

int world_to_cell_x(float x_mm)
{
    int coord = (int)floorf(
        (x_mm + MAP_ORIGIN_X_MM)
        / CELL_SIZE_MM);
    if (coord < 0) {
        return 0;
    } else if (coord >= MAP_WIDTH) {
        return MAP_WIDTH;
    } else {
        return coord;
    }

}

int world_to_cell_y(float y_mm)
{
    int coord = (int)floorf(
        (y_mm + MAP_ORIGIN_Y_MM)
        / CELL_SIZE_MM
    );
    if (coord < 0) {
        return 0;
    } else if (coord >= MAP_HEIGHT) {
        return MAP_HEIGHT;
    } else {
        return coord;
    }
}

void update_self() {
    self_x = world_to_cell_x(pose_get_x_mm());
    self_y = world_to_cell_y(pose_get_y_mm());
}

void map_init() {
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            WEIGHT_MAP[x][y] = 0;
            OBSTACLE_MAP[x][y] = 0;
        }
    }
    for (int i = 0; i < n; i++) {
        int x = starting_weight_estimates[i][0];
        int y = starting_weight_estimates[i][1];
        WEIGHT_MAP[x][y] = CONF_SCALE; // full confidence (1.000), not raw "1"
    }
    MAP_ORIGIN_X_MM = pose_get_x_mm();
    MAP_ORIGIN_Y_MM = pose_get_y_mm();

    home_x = world_to_cell_x(pose_get_x_mm());
    home_y = world_to_cell_y(pose_get_y_mm());
}

void apply_decay() {
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            // int32_t intermediate: max magnitude here is 1000 * 999 = 999000,
            // comfortably inside int32_t, so no overflow before the divide.
            WEIGHT_MAP[x][y] = (uint16_t)(((int32_t)WEIGHT_MAP[x][y] * DECAY_WEIGHT_PERMILLE) / 1000);

            int16_t obs = OBSTACLE_MAP[x][y];
            if (obs > 0) {
                OBSTACLE_MAP[x][y] = (int16_t)(((int32_t)obs * DECAY_OBSTACLE_PERMILLE) / 1000);
            } else {
                OBSTACLE_MAP[x][y] = (int16_t)(((int32_t)obs * DECAY_FREE_PERMILLE) / 1000);
            }
        }
    }
}

void arena_mirroring()
{
    for (int x = 0; x < MAP_WIDTH / 2; x++)
    {
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            int mirror_x = MAP_WIDTH - 1 - x;

            int16_t left  = OBSTACLE_MAP[x][y];
            int16_t right = OBSTACLE_MAP[mirror_x][y];

            if (abs(left) > abs(right))
            {
                OBSTACLE_MAP[mirror_x][y] =
                    (int16_t)(((int32_t)left * ARENA_MIRROR_PERMILLE) / 1000);
            }
            else if (abs(right) > abs(left))
            {
                OBSTACLE_MAP[x][y] =
                    (int16_t)(((int32_t)right * ARENA_MIRROR_PERMILLE) / 1000);
            }
        }
    }
}

void check_surroundings() {
    // check the 8 surrounding cells for obstacles
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;

            int x = self_x + dx;
            int y = self_y + dy;

            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                if (OBSTACLE_MAP[x][y] > 0) {
                    OBSTACLE_MAP[x][y] = CONF_SCALE;
                } else {
                    OBSTACLE_MAP[x][y] = -CONF_SCALE;
                }
            }
        }
    }
}

void update_map_arrays() {
    // TODO: needs your sensor-fusion API -- see note below
}

void calc_frontier() {
    // TODO: see note below for a proposed implementation
}

void add_obstacle_evidence(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= MAP_WIDTH ||
        cell_y < 0 || cell_y >= MAP_HEIGHT)
    {
        return;
    }

    OBSTACLE_MAP[cell_x][cell_y] = CONF_SCALE;
}


void add_free_evidence(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= MAP_WIDTH ||
        cell_y < 0 || cell_y >= MAP_HEIGHT)
    {
        return;
    }

    OBSTACLE_MAP[cell_x][cell_y] = -CONF_SCALE;
}

void update_obstacle_map(int distance_mm, float angle_deg, float sensor_cone_deg)
{
    bool hit = true;
    if (distance_mm <= 0)
    {
        distance_mm = 800; // max range of the sensor
        hit = false;
    }

    float angle = angle_deg * PI / 180.0;

    // Sensor position relative to robot ICR
    float sensor_x = 125.0 + 90.0 * cos(angle);
    float sensor_y = 90.0 * sin(angle);

    // Convert sensor position to world coordinates
    float heading = pose_get_heading_deg() * PI / 180.0;

    float sensor_world_x =
        pose_get_x_mm()
        + sensor_x * cos(heading)
        - sensor_y * sin(heading);

    float sensor_world_y =
        pose_get_y_mm()
        + sensor_x * sin(heading)
        + sensor_y * cos(heading);

    // Beam direction in world coordinates
    float beam_heading =
        heading + angle;

    float hit_world_x =
        sensor_world_x
        + distance_mm * cos(beam_heading);

    float hit_world_y =
        sensor_world_y
        + distance_mm * sin(beam_heading);

    // Convert hit position to map cells
    int hit_cell_x = world_to_cell_x(hit_world_x);
    int hit_cell_y = world_to_cell_y(hit_world_y);

    // Mark cells along ray as free
    int number_steps = distance_mm / CELL_SIZE_MM;

    for (int i = 0; i < number_steps; i++)
    {
        float fraction = (float)i / number_steps;

        float x =
            sensor_world_x
            + (hit_world_x - sensor_world_x) * fraction;

        float y =
            sensor_world_y
            + (hit_world_y - sensor_world_y) * fraction;

        int cell_x = world_to_cell_x(x);
        int cell_y = world_to_cell_y(y);

        // Don't mark the endpoint as free
        if (cell_x == hit_cell_x &&
            cell_y == hit_cell_y)
        {
            break;
        }

        add_free_evidence(cell_x, cell_y);
    }

    if(hit){
        // Endpoint is an obstacle
        add_obstacle_evidence(
            hit_cell_x,
            hit_cell_y
        );
    }
}


void add_weight_evidence(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= MAP_WIDTH ||
        cell_y < 0 || cell_y >= MAP_HEIGHT)
    {
        return;
    }

    WEIGHT_MAP[cell_x][cell_y] = 1000;
}

void remove_weight_evidence(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= MAP_WIDTH ||
        cell_y < 0 || cell_y >= MAP_HEIGHT)
    {
        return;
    }

    WEIGHT_MAP[cell_x][cell_y] = 0;
}

void update_weight_map(int distance_mm, float angle_deg, int distance_above_mm = -1)
{
    if (distance_above_mm == -1){
        if (distance_mm > 200)
        {
            return; // beyond middle sensor threshold, ignore
        }
    } else if (distance_mm <= 0)
    {
        return; // no weight detected

    } else if (distance_above_mm <= 0) 
    {
        if(distance_mm > 200 || distance_mm <= 0)
        {
            return; // beyond middle sensor threshold, ignore
        }
    }
    else if ((distance_mm - distance_above_mm) < 20)
    {
        return; // false positive - wall
    }

    float angle = angle_deg * PI / 180.0;

    float sensor_x = 125.0 + 90.0 * cos(angle);
    float sensor_y = 90.0 * sin(angle);

    float heading = pose_get_heading_deg() * PI / 180.0;

    float sensor_world_x =
        pose_get_x_mm()
        + sensor_x * cos(heading)
        - sensor_y * sin(heading);

    float sensor_world_y =
        pose_get_y_mm()
        + sensor_x * sin(heading)
        + sensor_y * cos(heading);

    float beam_heading = heading + angle;

    float hit_world_x =
        sensor_world_x
        + distance_mm * cos(beam_heading);

    float hit_world_y =
        sensor_world_y
        + distance_mm * sin(beam_heading);

    int hit_cell_x = world_to_cell_x(hit_world_x);
    int hit_cell_y = world_to_cell_y(hit_world_y);

    int number_steps = distance_mm / CELL_SIZE_MM;

    for (int i = 0; i < number_steps; i++)
    {
        float fraction = (float)i / number_steps;

        float x =
            sensor_world_x
            + (hit_world_x - sensor_world_x) * fraction;

        float y =
            sensor_world_y
            + (hit_world_y - sensor_world_y) * fraction;

        int cell_x = world_to_cell_x(x);
        int cell_y = world_to_cell_y(y);

        if (cell_x == hit_cell_x &&
            cell_y == hit_cell_y)
        {
            break;
        }

        if (cell_x >= 0 && cell_x < MAP_WIDTH &&
            cell_y >= 0 && cell_y < MAP_HEIGHT)
        {
            WEIGHT_MAP[cell_x][cell_y] = 0;
        }
    }


    add_weight_evidence(
        hit_cell_x,
        hit_cell_y
    );

}

void interpret_tof()
{
    update_obstacle_map(tof_get_distance(NAV_OUTER_LEFT), 40.0, 10.0);
    update_obstacle_map(tof_get_distance(NAV_INNER_LEFT),  15.0, 10.0);
    update_obstacle_map(tof_get_distance(NAV_INNER_RIGHT), -15.0, 10.0);
    update_obstacle_map(tof_get_distance(NAV_OUTER_RIGHT), -40.0, 10.0);

    update_weight_map(tof_get_distance(WEIGHT_LEFT_BOTTOM), -15.0,  tof_get_distance(WEIGHT_LEFT_TOP));
    update_weight_map(tof_get_distance(WEIGHT_RIGHT_BOTTOM), 15.0, tof_get_distance(WEIGHT_RIGHT_TOP));
    update_weight_map(tof_get_distance(WEIGHT_MIDDLE), 0.0);
}

// void interpret_ultrasonic() {
//     update_obstacle_map(ultrasonic_get_distance(0), 90.0, 20.0);
//     update_obstacle_map(ultrasonic_get_distance(1), -90.0, 20.0);
// }


void print_weight_map()
{
    Serial2.println("WEIGHT_MAP_START");

    for (int y = 0; y < MAP_HEIGHT; y++)
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            Serial2.print(WEIGHT_MAP[x][y]);

            if (x < MAP_WIDTH - 1)
                Serial2.print(",");
        }

        Serial2.println();
    }

    Serial2.println("WEIGHT_MAP_END");


    Serial2.println("OBSTACLE_MAP_START");

    for (int y = 0; y < MAP_HEIGHT; y++)
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            Serial2.print(OBSTACLE_MAP[x][y]);

            if (x < MAP_WIDTH - 1)
                Serial2.print(",");
        }

        Serial2.println();
    }

    Serial2.println("OBSTACLE_MAP_END");
    Serial2.println("Current position");
    Serial2.print(self_x); 
    Serial2.print(",");
    Serial2.print(self_y);
    Serial2.println();

    Serial2.println("TOF readings");
    Serial2.print(tof_get_distance(NAV_OUTER_LEFT));
    Serial2.print(",");
    Serial2.print(tof_get_distance(NAV_INNER_LEFT));
    Serial2.print(",");
    Serial2.print(tof_get_distance(NAV_INNER_RIGHT));
    Serial2.print(",");
    Serial2.print(tof_get_distance(NAV_OUTER_RIGHT));
    Serial2.print(",");
    Serial2.print(tof_get_distance(WEIGHT_LEFT_TOP));
    Serial2.print(",");
    Serial2.print(tof_get_distance(WEIGHT_RIGHT_TOP));
    Serial2.print(",");
    Serial2.print(tof_get_distance(WEIGHT_LEFT_BOTTOM));
    Serial2.print(",");
    Serial2.print(tof_get_distance(WEIGHT_RIGHT_BOTTOM));
    Serial2.print(",");
    Serial2.print(tof_get_distance(WEIGHT_MIDDLE));
    Serial2.println();

    Serial2.println("Heading");
    Serial2.print(pose_get_heading_deg());
    Serial2.println();
}


void map_update()
{
    iteration += 1;
    apply_decay();
    update_self();
    interpret_tof();
    arena_mirroring();
    check_surroundings();
    calc_frontier();
    if(max_iter < iteration) {
        print_weight_map();
        iteration = 0;
    }

}