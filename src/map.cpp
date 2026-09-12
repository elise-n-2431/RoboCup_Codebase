#include <stdint.h>
#include <HardwareSerial.h>
#include "pose.h"
#include <list>
#include <cstdint>
#include <cmath>
#include "inputs/tof_expander.h"

const int CELL_SIZE_MM = 50;

const int MAP_WIDTH = 98;
const int MAP_HEIGHT = 48;

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
int starting_weight_estimates[n][2] = {{4, 5}, {7, 9}, {7, 9}};

// Decay factors as integer "permille" (parts per thousand), e.g. 998 == 0.998.
// Applied as: new_val = (val * PERMILLE) / 1000, all in integer math.
const int32_t DECAY_WEIGHT_PERMILLE   = 999;
const int32_t DECAY_OBSTACLE_PERMILLE = 998; 
const int32_t DECAY_FREE_PERMILLE     = 999; 

const int32_t ARENA_MIRROR_PERMILLE = 200; 

int world_to_cell_x(float x_mm)
{
    return (int)floorf(
        (x_mm + MAP_ORIGIN_X_MM)
        / CELL_SIZE_MM
    );
}

int world_to_cell_y(float y_mm)
{
    return (int)floorf(
        (y_mm + MAP_ORIGIN_Y_MM)
        / CELL_SIZE_MM
    );
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

void arena_mirroring() {
    if (self_x > MAP_WIDTH / 2) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            for (int y = 0; y < MAP_HEIGHT; y++) {
                if (OBSTACLE_MAP[x][y] > -OBSTACLE_UNKNOWN_BAND && OBSTACLE_MAP[x][y] < OBSTACLE_UNKNOWN_BAND) {
                    int mirror_x = MAP_WIDTH - 1 - x;
                    OBSTACLE_MAP[x][y] = (int16_t)(((int32_t)OBSTACLE_MAP[mirror_x][y] * ARENA_MIRROR_PERMILLE) / 1000);
                }
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

void interpret_tof() {
    int distance_mm_LL = tof_get_distance(0);
    int distance_mm_L = tof_get_distance(5);
    int distance_mm_R = tof_get_distance(6);
    int distance_mm_RR = tof_get_distance(7);

    Serial.print("TOF distances: ");
    Serial.print(distance_mm_LL);
    Serial.print(", ");
    Serial.print(distance_mm_L);
    Serial.print(", ");
    Serial.print(distance_mm_R);
    Serial.print(", ");
    Serial.print(distance_mm_RR);
}

void interpret_ultrasonic() {

}

void map_update() {
    apply_decay();
    update_self();
    interpret_tof();
    interpret_ultrasonic();
    update_map_arrays();
    arena_mirroring();
    check_surroundings();
    calc_frontier();
}
