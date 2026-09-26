#include <stdint.h>
#include "path_finding.h"
#include "map.h"
#include "pose.h"
#include <queue>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

const int MAP_WIDTH = 97 + 4; // 2 cells at each extrema for walls
const int MAP_HEIGHT = 49 + 4;

/* Priority Queue */

int parent(int i) { return (i - 1) / 2; }

int leftChild(int i) { return 2 * i + 1; }

int rightChild(int i) { return 2 * i + 2; }

void shiftUp(int i, vector<Pair> &arr) {
    while (i > 0 && arr[i].key < arr[parent(i)].key) {
        swap(arr[parent(i)], arr[i]);
        i = parent(i);
    }
}

void shiftDown(int i, vector<Pair> &arr, int size) {
    int minIndex = i;
    int l = leftChild(i);
    if (l < size && arr[l].key < arr[minIndex].key) minIndex = l;
    int r = rightChild(i);
    if (r < size && arr[r].key < arr[minIndex].key) minIndex = r;

    if (i != minIndex) {
        swap(arr[i], arr[minIndex]);
        shiftDown(minIndex, arr, size);
    }
}

void insert(Pair p, vector<Pair> &arr) {
    arr.push_back(p);
    shiftUp(arr.size() - 1, arr);
}

Pair pop(vector<Pair> &arr) {
    int size = arr.size();
    Pair result = arr[0];
    arr[0] = arr[size - 1];
    arr.pop_back();
    shiftDown(0, arr, arr.size());
    return result;
}

Pair getMin(vector<Pair> &arr) {
    return arr[0];
}

bool contains(Node p, vector<Pair> &arr) {
    for (int i = 0; i < arr.size(); i++) {
        if (arr[i].node == p) return true;
    }
    return false;
}

int findIndex(Node p, vector<Pair> &arr) {
    for (int i = 0; i < arr.size(); i++) {
        if (arr[i].node == p) return i;
    }
    return -1; // not found
}

void update(Node p, Key newKey, vector<Pair> &arr) {
    int i = findIndex(p, arr);
    Key oldKey = arr[i].key;
    arr[i].key = newKey;
    if (newKey < oldKey) shiftUp(i, arr);
    else shiftDown(i, arr, arr.size());
}

void remove(Node p, vector<Pair> &arr) {
    int i = findIndex(p, arr);
    int last = arr.size() - 1;
    arr[i] = arr[last];
    arr.pop_back();
    if (i < (int)arr.size()) {
        shiftUp(i, arr);
        shiftDown(i, arr, arr.size());
    }
}

/* Priority Queue Ends*/

// D STAR LITE: searches from start node to goal node

Node start; // changes when robot moves
Node goal;
Node last;
int k_m;
vector<Pair> U; // custom priority queue
uint16_t RHS[MAP_WIDTH][MAP_HEIGHT]; // "right hand side" 
uint16_t G[MAP_WIDTH][MAP_HEIGHT];  // current shortest cost to reach start from goal
Node S[MAP_WIDTH * MAP_HEIGHT];
bool goal_reached = false;

Key Keys[5];

int g(Node p) {
    return G[p.x][p.y];
}

int rhs(Node p) {
    return RHS[p.x][p.y];
}

float heuristic(Node p, Node q) {
    // distance between 2 points (euclidean)
    return std::sqrt((p.x - q.x)^2 + (p.y - q.y)^2);

}

int cost(Node p, Node q) {
    if (!check_free(p.x, p.y) || !check_free(q.x, q.y)) {     // check map location
        return infinity();
    } else {
        return 1;
    }
}

bool consistent(Node p) {
    return rhs(p) == g(p);
}

vector<Node> neighbours(Node centre) {
    vector<Node> nodes;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) {
                continue; // skip centre
            }

            Node n = {centre.x + i, centre.y + j};
            if (n.x >= MAP_WIDTH || n.x < 0 || n.y >= MAP_HEIGHT || n.y < 0) {
                continue;
            } else {
                nodes.push_back(n); // add to list
            }
        }
    }
    return nodes;
}

Key calculate_key(Node s) { // s is self
    int cost_1 = std::min(G[s.x][s.y], RHS[s.x][s.y]) + heuristic(start, s) + k_m; // reprioritises values that have becvome locally inconsistent
    int cost_2 = std::min(G[s.x][s.y], RHS[s.x][s.y]);
    Key temp {cost_1, cost_2};
    return temp;
}


void path_init() {
    U.clear(); // reset U
    k_m = 0; // distance from start position

    start.x = world_to_cell_x(pose_get_x_mm());
    start.y = world_to_cell_y(pose_get_y_mm());

    goal.x = world_to_cell_x(2500);
    goal.y = world_to_cell_y(2500);

    for (int x = 0; x < MAP_WIDTH; x++)
        for (int y = 0; y < MAP_HEIGHT; y++) {
            RHS[x][y] = infinity;
            G[x][y] = infinity;
        }
    RHS[goal.x][goal.y] = 0;
    Pair goal_pair = {goal, calculate_key(goal)};
    insert(goal_pair, U);
}

void update_node(Node p) {
    if (G[p.x][p.y] != RHS[p.x][p.y] && contains(p, U)) {
        Key new_key = calculate_key(p);
        update(p, new_key, U);

    } else if (G[p.x][p.y] != RHS[p.x][p.y] && !contains(p, U)) {
        Pair new_pair = {p, calculate_key(p)};
        insert(new_pair, U);

    } else if (G[p.x][p.y] == RHS[p.x][p.y] && contains(p, U)) {
        remove(p, U);
        
    }
}

void compute_shortest_path() {
    Pair top = getMin(U);

    while (top.key < calculate_key(start) || !consistent(start)) {
        Key k_old = top.key;
        Pair u = pop(U);
        if (k_old < u.key) { // label this
            insert(u, U);

        } else if (g(u.node) > rhs(u.node)) { 
            G[u.node.x][u.node.y] = rhs(u.node);
            for (Node s : neighbours(u.node)) {
                update_node(s);
            }
        } else {
            G[u.node.x][u.node.y] = infinity;
            for (Node s : neighbours(u.node)) {
                update_node(s);
            }
            update_node(u.node);
        }
    }
}

Node choose_min_neighbour() {
    Node minimum = neighbours(start)[0];
    int min_cost = infinity;

    for (Node neighbour: neighbours(start)) {
        int n_cost = cost(start, neighbour) + g(start);
        
        if (min_cost > n_cost) {
            minimum = neighbour;
            min_cost = n_cost;
        }
    }

    return minimum;
}

void main() {
    last = start;
    path_init();
    compute_shortest_path();

    if (start != goal) {
        start = choose_min_neighbour();
        // physically move the robot to this neighbour
        goal_reached = false;
    }else {
        goal_reached = true;
    }
}


void main2() {
    if (goal_reached) return;

    if (!changed_cells.empty()) {
        k_m = k_m + heuristic(last, start);
        last = start;
        for (Node c : changed_cells) {
            for (Node n : neighbours(c)) {
                update_node(n);
            }
        }
        compute_shortest_path();
        changed_cells.clear();
    }
}
