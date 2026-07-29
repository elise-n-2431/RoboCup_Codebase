#include <stdint.h>
#include <HardwareSerial.h>

const int MAP_WIDTH = 10;
const int MAP_HEIGHT = 10;

uint8_t map[MAP_WIDTH][MAP_HEIGHT];

// 0: unexplored
// 1: empty
// 2: obstacle
// 3: frontier
// 4: self
// 5: home
// 6:
// 7: 

int self_x = 0;
int self_y = 0;

void update_self(int d_x, int d_y) {
    self_x += d_x;
    self_y += d_y;
}

void display_map() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {

            // Draw robot position over the map
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
                    Serial2.print('R');  // self
                    break;
                case 5:
                    Serial2.print('H');  // home
                    break;
                default:
                    Serial2.print('X');
                    break;
            }
        }

        Serial2.println(); // next row
    }

    Serial2.println(); // spacing between maps
}