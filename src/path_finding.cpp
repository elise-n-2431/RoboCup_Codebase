#include <stdint.h>
#include "path_finding.h"
#include "map.h"
#include "pose.h"
#include <queue>
#include <iostream>
#include <algorithm>

const int MAP_WIDTH = 97 + 4; // 2 cells at each extrema for walls
const int MAP_HEIGHT = 49 + 4;

// D STAR LITE: searches from start node to goal node
struct Node { // represent possible positions
    int x;
    int y;
    // Edges [Edge];
};


struct Edge { // paths between nodes
    Node a;
    Node b;
    int cost;
    
};

struct Key { // priority?
    int x;
    int y;

};

struct Pair {
    Node node;
    Key key;
};

Node start;
Node goal;
Node last;
int k_m = 0;
std::priority_queue<Pair> U; // potentially set up my own queuing alg
uint16_t RHS[MAP_WIDTH][MAP_HEIGHT]; // "right hand side" 
uint16_t G[MAP_WIDTH][MAP_HEIGHT];  // current shortest cost to reach start from goal



Key Keys[5];

int g(Node p) {
    return G[p.x][p.y];
}

int rhs(Node p) {
    return RHS[p.x][p.y];
}

float heuristic(Node p, Node q) {
    // distance between 2 points (manhattan)
    return std::sqrt((p.x - q.x)^2 + (p.y - q.y)^2);

}

float cost(Node p, Node q) {
    if (check_free(p.x, p.y) || check_free(q.x, q.y)) {     // check map location
        return infinity();
    } else {
        return heuristic(p, q);
    }
}


Key calculate_key(Node s) { // s is self
    int cost_1 = std::min(G[s.x][s.y], RHS[s.x][s.y]) + heuristic(start, s) + km; // don't understand k_m (accumulation)
    int cost_2 = std::min(G[s.x][s.y], RHS[s.x][s.y]);
    Key temp {cost_1, cost_2};
    return temp
}


void path_init() {
    start.x = world_to_cell_x(pose_get_x_mm());
    start.y = world_to_cell_y(pose_get_y_mm());

    goal.x = world_to_cell_x(2500);
    goal.y = world_to_cell_y(2500);

    RHS[goal.x][goal.y] = 0;




}

/* Only pseudo code from here */

void update_node(Node p) {
if (G[p.x][p.y] != RHS[p.x][p.y] && p in U) { // U is priority queue
    // update the key of p in priority queue
} else if (G[p.x][p.y] != RHS[p.x][p.y] && p NOT in U) {
    // insert the node, key pair into the end of the priority queue
} else if (G[p.x][p.y] == RHS[p.x][p.y] && p in U) {
    // remove the node, key pair from the priority queue
}
}


void compute_shortest_path() {
    while (U.top_key < calculate_key(start) || RHS[start.x][start.y] > G[start.x][start.y]) {
        Node p = U.top_node;
        int k_old = U.top_key;
        int k_new = calculate_key(p);
        if (k_old < k_new) {
            U[p] = k_new // replace key value for node in array
        } else if (G[p] > RHS[p]) {
            G[p] = RHS[p];
            U.remove(node)
            for ( all s element of Pred U???) {
                if (s != goal) {
                    RHS[s] = std::min()
                    ...
                }
                update_node(s);
            }
        } else {
            int g_old = g(p);
            g(p) = infinity;
            for (all s in predeccessors of p which are also in U) {
                if(rhs(s) == cost(s, p) + g_old) {
                    if (s != goal) {
                        rhs(s) = std::min() // what does this notation mean, min cost path?
                    }
                }
                update_node(s);
            } 
        }
    }
}


void main() {
    Node root = start;
    path_init();
    compute_shortest_path();

    while (start != goal) {
        cheapest_child = lowest cost of sucessors and cost to get to sucessors
        root = cheapest child;
        // if any edge costs change (use flags?)
            k_m = k_m + heuristic(last, root)

    }



}

