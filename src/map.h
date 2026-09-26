
//
// Created by elise on 29/07/2026.
//

#ifndef MAP_H
#define MAP_H



void map_init();
void map_update();
int get_frontier_x();
int get_frontier_y();
float get_frontier_world_x_mm();
float get_frontier_world_y_mm();
void change_print_it();
void send_map_data();
void print_target();
bool cell_too_close_to_obstacle(int, int);
void find_frontier();
void calc_frontier_target();
float get_front_clearance_mm();
bool get_frontier_target(float &x_mm, float &y_mm);

bool check_free(int x, int y);
bool check_obstacle(int x, int y);

int world_to_cell_x(float x_mm);
int world_to_cell_y(float y_mm);

float cell_to_world_x(int x);
float cell_to_world_y(int y);

#endif