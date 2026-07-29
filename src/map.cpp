#include <stdint.h>
#include <HardwareSerial.h>

const int MAP_WIDTH = 10;
const int MAP_HEIGHT = 10;

// Figure out conversion rate

uint8_t map[MAP_WIDTH][MAP_HEIGHT];

// 0: unexplored
// 1: empty
// 2: obstacle
// 3: frontier
// 4: self
// 5: home
// 6: weight
// 7: ramp

int self_x = 0;
int self_y = 0;

void update_self(int d_x, int d_y) {
    if (abs(d_x) >= 1) {
        self_x += 1;
    } if (abs(d_y) >= 1) {
        self_y += 1;
    }
}

void display_map() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {

            if (x == self_x && y == self_y) {
                Serial2.print('R');
                continue;
            }

            switch (map[x][y]) {
                case 0:
                    Serial2.print('?');  // unexplored
                    break;
                case 1:
                    Serial2.print('.');  // empty
                    break;
                case 2:
                    Serial2.print('#');  // obstacle
                    break;
                case 3:
                    Serial2.print('F');  // frontier
                    break;
                case 4:
                    Serial2.print('S');  // self
                    break;
                case 5:
                    Serial2.print('H');  // home
                    break;
                case 6:
                    Serial2.print('W');  // weight
                    break;
                case 7:
                    Serial2.print('R');   // ramp
                default:
                    Serial2.print('X');
                    break;
            }
        }

        Serial2.println(); // next row
    }

    Serial2.println(); // spacing between maps
}


float pose_x_mm = 0, pose_y_mm = 0, heading_rad = 0;

void update_pose(float d_x_mm, float d_y_mm, float d_heading_rad) {
    heading_rad += d_heading_rad;
    // rotate the flow-sensor's local displacement into map frame
    pose_x_mm += d_x_mm * cos(heading_rad) - d_y_mm * sin(heading_rad);
    pose_y_mm += d_x_mm * sin(heading_rad) + d_y_mm * cos(heading_rad);
}


int cell_x = (int)((pose_x_mm + world_x_mm) / CELL_SIZE_MM);
int cell_y = (int)((pose_y_mm + world_y_mm) / CELL_SIZE_MM);
if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {
    map[cell_x][cell_y] = 2; // obstacle
}