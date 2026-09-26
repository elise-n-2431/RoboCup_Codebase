//
// Created by elise on 25/09/2026.
//
#include <stdint.h>
#include "map.h"
#include "pose.h"
#include <queue>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

#ifndef PATH_FINDING_H
#define PATH_FINDING_H

struct Node { // represent possible positions
    int x;
    int y;

    bool operator==(const Node &other) const {
        return (x == other.x) && (y == other.y);
    }

    bool operator!=(const Node &other) const {
        return !(x == other.x && y == other.y);
    }

};


struct Key {
    float k1; // priority value
    float k2; // backup priority for comparison with equal k1

    bool operator<(const Key &other) const {
        if (k1 != other.k1) return k1 < other.k1;
        return k2 < other.k2;
    }

    bool operator<=(const Key &other) const {
        return !(other < *this);
    }
};

struct Pair {
    Node node;
    Key key;
};

extern vector<Node> changed_cells;

bool path_init();
float g(Node p);
float rhs(Node p);
float heuristic(Node p, Node q);
float cost(Node p, Node q);
bool consistent(Node p);
vector<Node> neighbours(Node centre);
Key calculate_key(Node s);
void update_node(Node p);
void compute_shortest_path();
Node choose_min_neighbour();

void path_notify_cell_changed(int x, int y);
void path_update();
bool path_get_next_waypoint(float &x_mm, float &y_mm);

void path_reset();
void main1();
void main2();



#endif