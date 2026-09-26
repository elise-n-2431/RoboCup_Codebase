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
struct Node { // represent possible positions
    int x;
    int y;
    bool operator==(const Node &other) const {
        return (x == other.x) && (y == other.y);
    }
    // Edges [Edge];
};


struct Edge { // paths between nodes
    Node a;
    Node b;
    int cost;
    
};

struct Key {
    int k1; // priority value
    int k2; // backup priority for comparison with equal k1

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

Node start; // changes when robot moves
Node goal;
Node last;
int k_m;
vector<Pair> U; // custom priority queue
uint16_t RHS[MAP_WIDTH][MAP_HEIGHT]; // "right hand side" 
uint16_t G[MAP_WIDTH][MAP_HEIGHT];  // current shortest cost to reach start from goal
Node S[MAP_WIDTH * MAP_HEIGHT];


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

bool consistent(Node p) {
    return rhs[p] == g[p];
}

[Node] neighbours(Node centre) {
    [Node] nodes = []
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            if (i != j) {
                list.append(Node[centre.x + i][centre.y + j])
            }
    }
    return nodes
}

Key calculate_key(Node s) { // s is self
    int cost_1 = std::min(G[s.x][s.y], RHS[s.x][s.y]) + heuristic(start, s) + km; // reprioritises values that have becvome locally inconsistent
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

    for (int i = 0; i < S.size(); i++) {
        Node s = S[i];
        RHS[s.x][s.y] = infinity;
        G[s.x][s.y] = infinity;
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
        int k_old = top.key;
        Node u = pop(U);
        Key u_key = calculate_key(u);

        if (k_old < u_key) {
            Pair u_pair = {u, new_key};
            insert(u_pair, U);

        } else if (g[p] > rhs[p]) {
            G[p.x][p.y] = rhs[p];
            for (int i = 0; )

            
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

