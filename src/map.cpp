#include <stdint.h>
#include <HardwareSerial.h>
#include "pose.h"
#include <list>
#include <cstdint>
#include <cmath>
#include "inputs/tof_expander.h"
#include <vector>
#include <numeric> // for std::accumulate


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

int max_iter = 100;
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
bool FRONTIER_MAP[MAP_WIDTH][MAP_HEIGHT];    // 0/1, 1=frontier

int self_x = 0; // define initial position in pose
int self_y = 0;

float MAP_ORIGIN_X_MM = 0;
float MAP_ORIGIN_Y_MM = 0;

int home_x = 0;
int home_y = 0;

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
const int32_t DECAY_WEIGHT_PERMILLE   = 995;
const int32_t DECAY_OBSTACLE_PERMILLE = 997; 
const int32_t DECAY_FREE_PERMILLE     = 997; 

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

    OBSTACLE_MAP[cell_x][cell_y] = -CONF_SCALE;
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
    for (int x = 0; x < MAP_WIDTH / 2; x++)
    {
        for (int y = 0; y < MAP_HEIGHT; y++)
        {
            int mirror_y = MAP_HEIGHT - 1 - y;

            int16_t left  = OBSTACLE_MAP[x][y];
            int16_t right = OBSTACLE_MAP[x][mirror_y];

            if (abs(left) > abs(right))
            {
                OBSTACLE_MAP[x][mirror_y] =
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

void update_obstacle_map(int distance_mm, float angle_deg, float sensor_cone_deg)
{
    bool hit = true;
    if (distance_mm == 0)
    {
        distance_mm = 1000; // max range of the sensor
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

void print_frontier_map_packed()
{
    Serial2.println("FRONTIER_MAP_START");

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
                if (byte < 0x10) Serial2.print('0');
                Serial2.print(byte, HEX);
                byte = 0;
                bit_count = 0;
            }
        }
    }

    if (bit_count > 0) // flush partial final byte
    {
        byte <<= (8 - bit_count);
        if (byte < 0x10) Serial2.print('0');
        Serial2.print(byte, HEX);
    }

    Serial2.println();
    Serial2.println("FRONTIER_MAP_END");
}

void print_obstacle_map_quantized()
{
    Serial2.println("OBSTACLE_MAP_START");

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
            Serial2.print(nibble, HEX);
        }
    }

    Serial2.println();
    Serial2.println("OBSTACLE_MAP_END");
}


void print_weight_map()
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

    Serial2.println("Target");
    Serial2.print(target.centre.x); 
    Serial2.print(",");
    Serial2.print(target.centre.y);
    Serial2.println();
}

void map_update()
{
    iteration += 1;

    apply_decay();
    update_self();
    interpret_tof();
    arena_mirroring();
    find_frontier();
    calc_frontier_target();

    if(max_iter < iteration) {
        print_weight_map();
        iteration = 0;
    }

}