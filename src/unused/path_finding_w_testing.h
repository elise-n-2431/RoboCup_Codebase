//
// Created by elise on 25/09/2026.
//
#include <stdint.h>
// #include "map.h"
// #include "pose.h"
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

vector<Node> changed_cells;


void path_init();
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
void main1();
void main2();
bool check_free_local(int, int);
void populate_map();
void update_map_display();
int main();

#endif