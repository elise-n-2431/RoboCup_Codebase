#include <stdint.h>
#include <HardwareSerial.h>
#include "pose.h"
#include <list>
#include <cstdint>
#include <cmath>
#include "inputs/tof_expander.h"
#include <vector>
#include <numeric> // for std::accumulate
#include "inputs/ultrasound.h"
#include "state_machine.h"
#include <iostream>
#include <queue>
using namespace std;


const int CELL_SIZE_MM = 50;

const int MAP_WIDTH = 97 + 4; // 2 cells at each extrema for walls
const int MAP_HEIGHT = 49 + 4;

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

int dist_o_l = 0;
int dist_o_r = 0;
int dist_i_l = 0;
int dist_i_r = 0;

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
bool FRONTIER_MAP[MAP_WIDTH][MAP_HEIGHT];    // 0/1, 1=frontier

int self_x = 0; // define initial position in pose
int self_y = 0;

float MAP_ORIGIN_X_MM = 0;
float MAP_ORIGIN_Y_MM = 0;

float x_min = -MAP_ORIGIN_X_MM;
float x_max = MAP_WIDTH * CELL_SIZE_MM - MAP_ORIGIN_X_MM;
float y_min = -MAP_ORIGIN_Y_MM;
float y_max = MAP_HEIGHT * CELL_SIZE_MM - MAP_ORIGIN_Y_MM;

int home_x = 0;
int home_y = 0;

static float g_cos_heading, g_sin_heading, heading;

std::vector<std::vector<int>> FRONTIER_GROUPS_X;
std::vector<std::vector<int>> FRONTIER_GROUPS_Y;

bool FRONTIER_VISITED[MAP_WIDTH][MAP_HEIGHT];

struct FrontierCentre {
    int x;
    int y;
};

struct FrontierTarget {
    bool valid;
    int group_index;
    FrontierCentre centre;
    float cost;
};

FrontierTarget target;

const int n = 3; // number of starting weight estimates
int starting_weight_estimates[n][2] = {{4, 5}, {7, 9}, {30, 30}};

// Decay factors as integer "permille" (parts per thousand), e.g. 998 == 0.998.
// Applied as: new_val = (val * PERMILLE) / 1000, all in integer math.
const int32_t DECAY_WEIGHT_PERMILLE   = 997;
const int32_t DECAY_OBSTACLE_PERMILLE = 1000; 
const int32_t DECAY_FREE_PERMILLE     = 999; 
const int32_t DECAY_OBSTACLE_IF_FREE  = 100;

const int32_t ARENA_MIRROR_PERMILLE = 200; 


int get_frontier_x() {
    return target.centre.x;
}

int get_frontier_y() {
    return target.centre.y;
}


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
    if (OBSTACLE_MAP[cell_x][cell_y] > 200) {
        OBSTACLE_MAP[cell_x][cell_y] -= DECAY_OBSTACLE_IF_FREE;
    } else {
        OBSTACLE_MAP[cell_x][cell_y] = -CONF_SCALE;
    }
}

void update_self() {
    self_x = world_to_cell_x(pose_get_x_mm());
    self_y = world_to_cell_y(pose_get_y_mm());

    for (int i =-2; i < 3; i++) { // account for size of robot, assume square shape
        for (int j =-2; j < 3; j++) {
            add_free_evidence(self_x + i, self_y + j);
        }
    }
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
    for (int x = 0; x < MAP_WIDTH; x++)
    {
        for (int y = 0; y < MAP_HEIGHT / 2; y++)
        {
            int mirror_y = MAP_HEIGHT - 1 - y;

            int16_t left  = OBSTACLE_MAP[x][y];
            int16_t right = OBSTACLE_MAP[x][mirror_y];

            if (abs(left) > abs(right) + 200)
            {
                OBSTACLE_MAP[x][mirror_y] =
                    (int16_t)(((int32_t)left * ARENA_MIRROR_PERMILLE) / 1000);
            }
            else if (abs(right) > abs(left) + 200)
            {
                OBSTACLE_MAP[x][y] =
                    (int16_t)(((int32_t)right * ARENA_MIRROR_PERMILLE) / 1000);
            }
        }
    }
}

// void check_surroundings() { // check surroundings aren't all walls - problematic
//     for (int i =-2; i < 3; i+=) { 
//         for (int j =-2; j < 3; j++) {
//             add_free_evidence(self_x + i, self_y + j);
//         }
//     }
// }

void find_frontier() {
    for (int x = 1; x < MAP_WIDTH - 1; x++)
    {
        for (int y = 1; y < MAP_HEIGHT - 1; y++)
        {
            if (OBSTACLE_MAP[x][y] > OBSTACLE_UNKNOWN_BAND || OBSTACLE_MAP[x][y] < -OBSTACLE_UNKNOWN_BAND)
            {
                FRONTIER_MAP[x][y] = false;
            }
            else if(self_x != x && self_y != y){ // unexplored space
                FRONTIER_MAP[x][y] = false;

                for (int i = -1; i <= 1; i++) {
                    for (int j = -1; j <= 1; j++) {
                        if (!(i == 0 && j == 0)) {
                            if (OBSTACLE_MAP[x + i][y + j] < -OBSTACLE_UNKNOWN_BAND) {
                                FRONTIER_MAP[x][y] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    // --------------------------------------------------
    // Group adjacent frontier cells (8-connectivity)
    // --------------------------------------------------

    FRONTIER_GROUPS_X.clear();
    FRONTIER_GROUPS_Y.clear();
    memset(FRONTIER_VISITED, 0, sizeof(FRONTIER_VISITED));

    static int16_t stack_x[MAP_WIDTH * MAP_HEIGHT];
    static int16_t stack_y[MAP_WIDTH * MAP_HEIGHT];

    for (int x = 1; x < MAP_WIDTH - 1; x++)
    {
        for (int y = 1; y < MAP_HEIGHT - 1; y++)
        {
            if (!FRONTIER_MAP[x][y] || FRONTIER_VISITED[x][y]) continue;

            std::vector<int> group_x;
            std::vector<int> group_y;

            int sp = 0;
            stack_x[sp] = x;
            stack_y[sp] = y;
            sp++;
            FRONTIER_VISITED[x][y] = true;

            while (sp > 0)
            {
                sp--;
                int cx = stack_x[sp];
                int cy = stack_y[sp];

                group_x.push_back(cx);
                group_y.push_back(cy);

                for (int i = -1; i <= 1; i++)
                {
                    for (int j = -1; j <= 1; j++)
                    {
                        if (i == 0 && j == 0) continue;

                        int nx = cx + i;
                        int ny = cy + j;

                        if (nx < 1 || nx >= MAP_WIDTH - 1 || ny < 1 || ny >= MAP_HEIGHT - 1) continue;

                        if (FRONTIER_MAP[nx][ny] && !FRONTIER_VISITED[nx][ny])
                        {
                            FRONTIER_VISITED[nx][ny] = true;
                            stack_x[sp] = nx;
                            stack_y[sp] = ny;
                            sp++;
                        }
                    }
                }
            }

            FRONTIER_GROUPS_X.push_back(group_x);
            FRONTIER_GROUPS_Y.push_back(group_y);
        }
    }
}

float cost(float distance, int size, float orientation) {
    float c1 = 1.0f;
    float c2 = 1.0f;
    float c3 = 1.0f;
    return c1 * distance - c2 * size + c3 * fabsf(orientation);
}

FrontierCentre get_frontier_centre(int group_index)
{
    const std::vector<int> &gx = FRONTIER_GROUPS_X[group_index];
    const std::vector<int> &gy = FRONTIER_GROUPS_Y[group_index];

    long sum_x = std::accumulate(gx.begin(), gx.end(), 0L);
    long sum_y = std::accumulate(gy.begin(), gy.end(), 0L);

    FrontierCentre centre;
    centre.x = (int)sum_x / gx.size();
    centre.y = (int)sum_y / gy.size();

    return centre;
}

void calc_frontier_target() {
    FrontierTarget best;
    best.valid = false;
    best.cost = INFINITY;

    for (int i = 0; i < (int)FRONTIER_GROUPS_X.size(); i++) {
        int size = FRONTIER_GROUPS_X[i].size();
        FrontierCentre centre = get_frontier_centre(i);

        float dx = centre.x - self_x;
        float dy = centre.y - self_y;
        float sqrd_distance = dx * dx + dy * dy;

        float xy_orientation = atan2f(dy, dx);
        float heading = pose_get_heading_deg() * PI / 180.0f;
        float relative_orientation = xy_orientation - heading;

        while (relative_orientation > PI)  relative_orientation -= 2.0f * PI;
        while (relative_orientation < -PI) relative_orientation += 2.0f * PI;

        float frontier_cost = cost(sqrd_distance, size, relative_orientation);

        if (frontier_cost < best.cost) {
            best.valid = true;
            best.group_index = i;
            best.centre = centre;
            best.cost = frontier_cost;
        }
    }
    target = best;
}

void update_obstacle_map(int distance_mm, float angle_deg, int sensor_x_pos = 125)
{
    bool hit = true;
    if (distance_mm <= 0)
    {
        distance_mm = 1000; // max range of the sensor
        hit = false;
    }
    if (distance_mm > 1000) 
    {
        distance_mm = 1000;
        hit = false;
    }

    float angle = angle_deg * PI / 180.0;

    // Sensor position relative to robot ICR
    float sensor_x = sensor_x_pos + 90.0 * cos(angle);
    float sensor_y = 90.0 * sin(angle);

    // Convert sensor position to world coordinates
    // float heading = pose_get_heading_deg() * PI / 180.0;

    float sensor_world_x =
        pose_get_x_mm()
        + sensor_x * g_cos_heading
        - sensor_y * g_sin_heading;

    float sensor_world_y =
        pose_get_y_mm()
        + sensor_x * g_sin_heading
        + sensor_y * g_cos_heading;

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
    
    // if (dstar_active) {
    //     updateNode(cell_x, cell_y);
    //     std::vector<std::pair<int,int>> nb;
    //     getNeighbors(cell_x, cell_y, nb);
    //     for (auto& p : nb) updateNode(p.first, p.second);
    // }
}

void remove_weight_evidence(int cell_x, int cell_y)
{
    if (cell_x < 0 || cell_x >= MAP_WIDTH ||
        cell_y < 0 || cell_y >= MAP_HEIGHT)
    {
        return;
    }

    WEIGHT_MAP[cell_x][cell_y] = 0;

    // if (dstar_active) {
    //     updateNode(cell_x, cell_y);
    //     std::vector<std::pair<int,int>> nb;
    //     getNeighbors(cell_x, cell_y, nb);
    //     for (auto& p : nb) updateNode(p.first, p.second);
    // }
}

// void update_weight_map(int distance_mm, float angle_deg, int distance_above_mm = -1)
// {
//     if (distance_above_mm == -1){
//         if (distance_mm > 200)
//         {
//             return; // beyond middle sensor threshold, ignore
//         }
//     } else if (distance_mm <= 0)
//     {
//         return; // no weight detected

//     } else if (distance_above_mm <= 0) 
//     {
//         if(distance_mm > 200 || distance_mm <= 0)
//         {
//             return; // beyond middle sensor threshold, ignore
//         }
//     }
//     else if ((distance_mm - distance_above_mm) < 20)
//     {
//         return; // false positive - wall
//     }

//     float angle = angle_deg * PI / 180.0;

//     float sensor_x = 125 + 90.0 * cos(angle);
//     float sensor_y = 90.0 * sin(angle);

//     float heading = pose_get_heading_deg() * PI / 180.0;

//     float sensor_world_x =
//         pose_get_x_mm()
//         + sensor_x * cos(heading)
//         - sensor_y * sin(heading);

//     float sensor_world_y =
//         pose_get_y_mm()
//         + sensor_x * sin(heading)
//         + sensor_y * cos(heading);

//     float beam_heading = heading + angle;

//     float hit_world_x =
//         sensor_world_x
//         + distance_mm * cos(beam_heading);

//     float hit_world_y =
//         sensor_world_y
//         + distance_mm * sin(beam_heading);

//     int hit_cell_x = world_to_cell_x(hit_world_x);
//     int hit_cell_y = world_to_cell_y(hit_world_y);

//     int number_steps = distance_mm / CELL_SIZE_MM;

//     for (int i = 0; i < number_steps; i++)
//     {
//         float fraction = (float)i / number_steps;

//         float x =
//             sensor_world_x
//             + (hit_world_x - sensor_world_x) * fraction;

//         float y =
//             sensor_world_y
//             + (hit_world_y - sensor_world_y) * fraction;

//         int cell_x = world_to_cell_x(x);
//         int cell_y = world_to_cell_y(y);

//         if (cell_x == hit_cell_x &&
//             cell_y == hit_cell_y)
//         {
//             break;
//         }

//         if (cell_x >= 0 && cell_x < MAP_WIDTH &&
//             cell_y >= 0 && cell_y < MAP_HEIGHT)
//         {
//             WEIGHT_MAP[cell_x][cell_y] = 0;
//         }
//     }

//     add_weight_evidence(
//         hit_cell_x,
//         hit_cell_y
//     );

// }


float raycast_to_arena_wall(float ox, float oy, float angle_rad)
{
    float x_min = -MAP_ORIGIN_X_MM;
    float x_max = MAP_WIDTH  * CELL_SIZE_MM - MAP_ORIGIN_X_MM;
    float y_min = -MAP_ORIGIN_Y_MM;
    float y_max = MAP_HEIGHT * CELL_SIZE_MM - MAP_ORIGIN_Y_MM;

    float dx = cos(angle_rad);
    float dy = sin(angle_rad);

    float t_best = INFINITY;

    // Check each of the 4 boundary lines, keep nearest positive-t hit
    // that actually falls within the rectangle's other axis.
    if (dx > 1e-6f) {
        float t = (x_max - ox) / dx;
        float y = oy + t * dy;
        if (t > 0 && y >= y_min && y <= y_max) t_best = min(t_best, t);
    } else if (dx < -1e-6f) {
        float t = (x_min - ox) / dx;
        float y = oy + t * dy;
        if (t > 0 && y >= y_min && y <= y_max) t_best = min(t_best, t);
    }
    if (dy > 1e-6f) {
        float t = (y_max - oy) / dy;
        float x = ox + t * dx;
        if (t > 0 && x >= x_min && x <= x_max) t_best = min(t_best, t);
    } else if (dy < -1e-6f) {
        float t = (y_min - oy) / dy;
        float x = ox + t * dx;
        if (t > 0 && x >= x_min && x <= x_max) t_best = min(t_best, t);
    }

    return isfinite(t_best) ? t_best : -1.0f;
}

const float WALL_CORRECTION_GAIN = 0.10f;   // start small, tune up
const float WALL_MATCH_TOLERANCE_MM = 60.0f; // reject if measured is way off predicted

static float g_correction_sum_x, g_correction_sum_y = 0;
static int g_correction_count = 0;


void try_wall_correction(int distance_mm, float angle_deg, int sensor_x_pos = 125)
{
    if (distance_mm <= 0 || distance_mm > 1000) return; // no confirmed hit

    float angle = angle_deg * PI / 180.0f;
    float sensor_x = sensor_x_pos + 90.0f * cos(angle);
    float sensor_y = 90.0f * sin(angle);

    float sensor_world_x = pose_get_x_mm() + sensor_x * g_cos_heading - sensor_y * g_sin_heading;
    float sensor_world_y = pose_get_y_mm() + sensor_x * g_sin_heading + sensor_y * g_cos_heading;

    float beam_heading = heading + angle;

    float expected = raycast_to_arena_wall(sensor_world_x, sensor_world_y, beam_heading);
    if (expected < 0) return;

    float residual = distance_mm - expected;

    // Gate: only trust this as a wall hit if it's close to the predicted
    // wall distance. A big residual means something else is in the way
    // (obstacle, robot, weight) -- not a wall, don't use it.
    if (fabsf(residual) > WALL_MATCH_TOLERANCE_MM) return;

    // Also gate on the hit actually landing near the map border, as a
    // second sanity check using your existing cell grid.
    int hit_cell_x = world_to_cell_x(sensor_world_x + expected * cos(beam_heading));
    int hit_cell_y = world_to_cell_y(sensor_world_y + expected * sin(beam_heading));
    bool near_border =
        hit_cell_x <= 1 || hit_cell_x >= MAP_WIDTH - 2 ||
        hit_cell_y <= 1 || hit_cell_y >= MAP_HEIGHT - 2;
    if (!near_border) return;

    g_correction_sum_x += residual * cos(beam_heading);
    g_correction_sum_y += residual * sin(beam_heading);
    g_correction_count++;
}


void map_correction()
{
    g_correction_sum_x = 0;
    g_correction_sum_y = 0;
    g_correction_count = 0;

    for (int i = -3; i < 4; i += 2) {
        try_wall_correction(dist_o_l, -40.0f + i);
        try_wall_correction(dist_i_l, -15.0f + i);
        try_wall_correction(dist_i_r,  15.0f + i);
        try_wall_correction(dist_o_r,  40.0f + i);
    }
    try_wall_correction(ultrasound_get_left_mm(),  -90.0f, 0);
    try_wall_correction(ultrasound_get_right_mm(),  90.0f, 0);

    if (g_correction_count > 0) {
        float dx = (g_correction_sum_x / g_correction_count) * WALL_CORRECTION_GAIN;
        float dy = (g_correction_sum_y / g_correction_count) * WALL_CORRECTION_GAIN;
        pose_apply_correction(dx, dy);
    }
}



void interpret_tof()
{
    for (int i = -3; i < 4; i += 2) {
        dist_o_l = tof_get_distance(NAV_OUTER_LEFT);
        update_obstacle_map(dist_o_l, -40.0 + i);
        dist_i_l = tof_get_distance(NAV_INNER_LEFT);
        update_obstacle_map(dist_i_l,  -15.0 + i);
        dist_i_r = tof_get_distance(NAV_INNER_RIGHT);
        update_obstacle_map(dist_i_r, 15.0 + i);
        dist_o_r = tof_get_distance(NAV_OUTER_RIGHT);
        update_obstacle_map(dist_o_r, 40.0 + i);
    }

    // update_weight_map(tof_get_distance(WEIGHT_LEFT_BOTTOM), -15.0,  tof_get_distance(WEIGHT_LEFT_TOP));
    // update_weight_map(tof_get_distance(WEIGHT_RIGHT_BOTTOM), 15.0, tof_get_distance(WEIGHT_RIGHT_TOP));
    // update_weight_map(tof_get_distance(WEIGHT_MIDDLE), 0.0);
}

void interpret_ultrasonic() { // multiple to get wide cone shape
    for (int i = 80; i < 101; i += 2) {
        update_obstacle_map(ultrasound_get_left_mm(), -i, 0);
        update_obstacle_map(ultrasound_get_right_mm(), i, 0);
    }

}

void print_frontier_map_packed()
{
    Serial.println("FRONTIER_MAP_START");

    uint8_t byte = 0;
    int bit_count = 0;

    for (int y = 0; y < MAP_HEIGHT; y++)
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            byte = (byte << 1) | (FRONTIER_MAP[x][y] ? 1 : 0);
            bit_count++;

            if (bit_count == 8)
            {
                if (byte < 0x10) Serial.print('0');
                Serial.print(byte, HEX);
                byte = 0;
                bit_count = 0;
            }
        }
    }

    if (bit_count > 0) // flush partial final byte
    {
        byte <<= (8 - bit_count);
        if (byte < 0x10) Serial.print('0');
        Serial.print(byte, HEX);
    }

    Serial.println();
    Serial.println("FRONTIER_MAP_END");
}

void print_obstacle_map_quantized()
{
    Serial.println("OBSTACLE_MAP_START");

    for (int y = 0; y < MAP_HEIGHT; y++)
    {
        for (int x = 0; x < MAP_WIDTH; x++)
        {
            int16_t val = OBSTACLE_MAP[x][y];
            if (val > CONF_SCALE)  val = CONF_SCALE;
            if (val < -CONF_SCALE) val = -CONF_SCALE;

            // scale to -7..7, rounding to nearest instead of truncating
            int32_t scaled = (int32_t)val * 7;
            int8_t q;

            if (scaled >= 0)
                q = (int8_t)((scaled + CONF_SCALE / 2) / CONF_SCALE);
            else
                q = (int8_t)((scaled - CONF_SCALE / 2) / CONF_SCALE);

            if (q > 7)  q = 7;
            if (q < -7) q = -7;

            // encode as 4-bit two's complement, print as one hex digit
            uint8_t nibble = (uint8_t)(q & 0x0F);
            Serial.print(nibble, HEX);
        }
    }

    Serial.println();
    Serial.println("OBSTACLE_MAP_END");
}

void send_map_data()
{
    // Serial2.println("WEIGHT_MAP_START");

    // for (int y = 0; y < MAP_HEIGHT; y++)
    // {
    //     for (int x = 0; x < MAP_WIDTH; x++)
    //     {
    //         Serial2.print(WEIGHT_MAP[x][y]);

    //         if (x < MAP_WIDTH - 1)
    //             Serial2.print(",");
    //     }

    //     Serial2.println();
    // }

    // Serial2.println("WEIGHT_MAP_END");

    print_obstacle_map_quantized();
       
    print_frontier_map_packed();

    Serial.println("Current position");
    Serial.print(self_x); 
    Serial.print(",");
    Serial.print(self_y);
    Serial.println();

    Serial.println("TOF readings");
    Serial.print(dist_o_l);
    Serial.print(",");
    Serial.print(dist_i_l);
    Serial.print(",");
    Serial.print(dist_i_r);
    Serial.print(",");
    Serial.print(dist_o_r);
    Serial.print(",");
    Serial.print(tof_get_weight_left_top());
    Serial.print(",");
    Serial.print(tof_get_weight_right_top());
    Serial.print(",");
    Serial.print(tof_get_weight_left_bottom());
    Serial.print(",");
    Serial.print(tof_get_weight_right_bottom());
    Serial.print(",");
    Serial.print(tof_get_weight_middle());
    Serial.println();

    Serial.println("Heading");
    Serial.print(pose_get_heading_deg());
    Serial.println();

    Serial.println("Target");
    Serial.print(target.centre.x); 
    Serial.print(",");
    Serial.print(target.centre.y);
    Serial.println();
}

// temp var to calc period
static uint32_t dbg_max_us = 0;
static uint32_t dbg_sum_us = 0;
static uint32_t dbg_calls  = 0;

bool homing_init = false;

void map_update()
{
    uint32_t t0 = micros();



    heading = pose_get_heading_deg() * PI / 180.0f;
    g_cos_heading = cosf(heading);
    g_sin_heading = sinf(heading);

    apply_decay();
    update_self();
    interpret_tof();
    interpret_ultrasonic();
    map_correction();
    arena_mirroring();
    find_frontier();
    calc_frontier_target();

    // NavState nav = getNavState();
    // if (nav == HOMING) {
    //     if (!homing_init) {
    //         initialize();
    //         computeShortestPath();
    //         homing_init = true;
    //     } else {
    //         if (self_x != last_self_x || self_y != last_self_y) {
    //             km += d_heuristic(last_self_x, last_self_y, self_x, self_y);
    //             last_self_x = self_x;
    //             last_self_y = self_y;
    //         }
    //         computeShortestPath();
    //     }
    // }



    // print_weight_map();


    // calculate period time for map

    uint32_t dt = micros() - t0;
    dbg_sum_us += dt;
    dbg_calls++;
    if (dt > dbg_max_us) dbg_max_us = dt;

    if (dbg_calls % 20 == 0) {
        Serial.print(F("map_update avg_us="));
        Serial.print(dbg_sum_us / dbg_calls);
        Serial.print(F(" max_us="));
        Serial.println(dbg_max_us);
    }
}


// D Star path finding for homing



// #include <set>
// #include <map>


// // ---- grid index / node storage --------------------------------------------
// static inline int d_idx(int x, int y) { return y * MAP_WIDTH + x; }

// struct Node {
//     float g   = INFINITY;
//     float rhs = INFINITY;
// };

// static Node NODES[MAP_WIDTH][MAP_HEIGHT];

// // ---- key type ---------------------------------------------------------------
// struct Key {
//     float k1, k2;
//     bool operator<(const Key& o) const {
//         if (k1 != o.k1) return k1 < o.k1;
//         return k2 < o.k2;
//     }
// };

// // ---- priority queue: supports insert / remove / contains / pop / top-key ---
// // (std::priority_queue can't do remove(), which updateNode() needs, so this
// // is a std::set keyed on (key,x,y) plus a lookup map for contains/remove.)
// struct QEntry {
//     Key key; int x, y;
//     bool operator<(const QEntry& o) const {
//         if (!(key.k1 == o.key.k1 && key.k2 == o.key.k2)) return key < o.key;
//         if (x != o.x) return x < o.x;
//         return y < o.y;
//     }
// };

// struct Queue {
//     std::set<QEntry> entries;
//     std::map<int, Key> node_key; // node index -> its current key, for contains/remove

//     void insert(int x, int y, Key k) {
//         entries.insert({k, x, y});
//         node_key[d_idx(x, y)] = k;
//     }
//     void remove(int x, int y) {
//         int i = d_idx(x, y);
//         auto it = node_key.find(i);
//         if (it == node_key.end()) return;
//         entries.erase({it->second, x, y});
//         node_key.erase(it);
//     }
//     bool contains(int x, int y) {
//         return node_key.count(d_idx(x, y)) > 0;
//     }
//     Key topKey() {
//         if (entries.empty()) return {INFINITY, INFINITY};
//         return entries.begin()->key;
//     }
//     void pop(int& x, int& y) {
//         auto it = entries.begin();
//         x = it->x; y = it->y;
//         node_key.erase(d_idx(x, y));
//         entries.erase(it);
//     }
//     bool empty() { return entries.empty(); }
// };

// static Queue pq;
// static float km = 0.0f;
// static int last_self_x, last_self_y;
// static bool dstar_active = false;

// static inline float d_heuristic(int ax, int ay, int bx, int by) {
//     int dx = abs(ax - bx), dy = abs(ay - by);
//     int dmin = min(dx, dy), dmax = max(dx, dy);
//     return dmax + 0.41421356f * dmin; // octile distance, 8-connected grid
// }

// // Cost of entering cell (x,y), derived from OBSTACLE_MAP you already maintain.
// static inline float getCostTo(int x, int y) {
//     if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return INFINITY;
//     int16_t v = OBSTACLE_MAP[x][y];
//     if (v > OBSTACLE_UNKNOWN_BAND)  return INFINITY; // confirmed obstacle
//     if (v < -OBSTACLE_UNKNOWN_BAND) return 1.0f; // confirmed free
//     return 5.0f;                                  // unknown -- passable but discouraged
// }

// static void getNeighbors(int x, int y, std::vector<std::pair<int,int>>& out) {
//     out.clear();
//     for (int i = -1; i <= 1; i++)
//         for (int j = -1; j <= 1; j++) {
//             if (i == 0 && j == 0) continue;
//             int nx = x + i, ny = y + j;
//             if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT)
//                 out.push_back({nx, ny});
//         }
// }

// // ---- calculateKey ------------------------------------------------------------
// Key calculateKey(int x, int y) {
//     float m = min(NODES[x][y].g, NODES[x][y].rhs);
//     return { m + d_heuristic(x, y, self_x, self_y) + km, m };
// }

// // ---- updateNode ---------------------------------------------------------------
// void updateNode(int x, int y) {
//     if (x == home_x && y == home_y) return; // start.rhs stays 0 forever

//     Node& n = NODES[x][y];
//     n.rhs = INFINITY;

//     std::vector<std::pair<int,int>> preds;
//     getNeighbors(x, y, preds);
//     for (auto& p : preds) {
//         float cand = NODES[p.first][p.second].g + getCostTo(x, y);
//         if (cand < n.rhs) n.rhs = cand;
//     }

//     if (pq.contains(x, y)) pq.remove(x, y);
//     if (n.g != n.rhs) pq.insert(x, y, calculateKey(x, y));
// }

// // ---- initialize -----------------------------------------------------------
// void initialize() {
//     for (int x = 0; x < MAP_WIDTH; x++)
//         for (int y = 0; y < MAP_HEIGHT; y++) {
//             NODES[x][y].g = INFINITY;
//             NODES[x][y].rhs = INFINITY;
//         }

//     km = 0.0f;
//     last_self_x = self_x;
//     last_self_y = self_y;

//     NODES[home_x][home_y].rhs = 0.0f;
//     pq.insert(home_x, home_y, calculateKey(home_x, home_y));

//     dstar_active = true;
// }

// // ---- computeShortestPath -----------------------------------------------------
// void computeShortestPath() {
//     while (!pq.empty() &&
//            ((pq.topKey() < calculateKey(self_x, self_y)) ||
//             (NODES[self_x][self_y].rhs != NODES[self_x][self_y].g))) {

//         int x, y;
//         pq.pop(x, y);
//         Node& n = NODES[x][y];

//         if (n.g > n.rhs) {
//             n.g = n.rhs;
//         } else {
//             n.g = INFINITY;
//             updateNode(x, y);
//         }

//         std::vector<std::pair<int,int>> succ;
//         getNeighbors(x, y, succ);
//         for (auto& s : succ) updateNode(s.first, s.second);
//     }
// }


// map_period_ms = max(ceil(max_us / 1000) * 3