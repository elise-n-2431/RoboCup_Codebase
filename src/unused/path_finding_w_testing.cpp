#include <stdint.h>
#include "path_finding.h"
// #include "map.h"
// #include "pose.h"
#include <queue>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

const int MAP_WIDTH = 97 + 4; // 2 cells at each extrema for walls
const int MAP_HEIGHT = 49 + 4;
int16_t  OBSTACLE_MAP_COPY[MAP_WIDTH][MAP_HEIGHT];
int CLEARANCE = 6;
int PENALTY_WEIGHT = 3;

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
float RHS[MAP_WIDTH][MAP_HEIGHT]; // "right hand side" 
float G[MAP_WIDTH][MAP_HEIGHT];  // current shortest cost to reach start from goal
bool goal_reached = false;
const int INF = 65535; // fits uint16_t

float g(Node p) {
    return G[p.x][p.y];
}

float rhs(Node p) {
    return RHS[p.x][p.y];
}

float heuristic(Node p, Node q) {
    // distance between 2 points (euclidean)
    return sqrt((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y));

}

float wallPenalty(Node p) {
    float minDist = 1e9;
    for (int x = 0; x < MAP_WIDTH; x++)
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (!OBSTACLE_MAP_COPY[x][y]) continue;
            float d = heuristic(p, {x, y});
            if (d < minDist) minDist = d;
        }
    if (minDist >= CLEARANCE) return 0;
    return (CLEARANCE - minDist) * PENALTY_WEIGHT;
}

float cost(Node p, Node q) {
    if (!check_free_local(p.x, p.y) || !check_free_local(q.x, q.y)) return INF;
    return heuristic(p, q) + wallPenalty(q);
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
    float cost_1 = std::min(G[s.x][s.y], RHS[s.x][s.y]) + heuristic(start, s) + k_m; // reprioritises values that have becvome locally inconsistent
    float cost_2 = std::min(G[s.x][s.y], RHS[s.x][s.y]);
    Key temp {cost_1, cost_2};
    return temp;
}


void path_init() {
    U.clear(); // reset U
    k_m = 0; // distance from start position

    start = {2, 1};
    goal = {70,50};
    // start.x = world_to_cell_x(pose_get_x_mm());
    // start.y = world_to_cell_y(pose_get_y_mm());

    // goal.x = world_to_cell_x(2500);
    // goal.y = world_to_cell_y(2500);

    for (int x = 0; x < MAP_WIDTH; x++)
        for (int y = 0; y < MAP_HEIGHT; y++) {
            RHS[x][y] = INF;
            G[x][y] = INF;
        }
    RHS[goal.x][goal.y] = 0;
    Pair goal_pair = {goal, calculate_key(goal)};
    insert(goal_pair, U);
}

void update_node(Node p) {
    if (!(p == goal)) {
        int min_rhs = INF;
        for (Node s : neighbours(p)) {
            int val = cost(p, s) + g(s);
            if (val < min_rhs) min_rhs = val;
        }
        RHS[p.x][p.y] = min_rhs;
    }

    if (G[p.x][p.y] != RHS[p.x][p.y] && contains(p, U)) {
        update(p, calculate_key(p), U);
    } else if (G[p.x][p.y] != RHS[p.x][p.y] && !contains(p, U)) {
        insert({p, calculate_key(p)}, U);
    } else if (G[p.x][p.y] == RHS[p.x][p.y] && contains(p, U)) {
        remove(p, U);
    }
}

void compute_shortest_path() {
    while (!U.empty() && (getMin(U).key < calculate_key(start) || !consistent(start))) {
        Pair top = getMin(U);
        Key k_old = top.key;
        Pair u = pop(U);
        Key u_key = calculate_key(u.node);

        if (k_old < u_key) {
            insert({u.node, u_key}, U);
        } else if (g(u.node) > rhs(u.node)) {
            G[u.node.x][u.node.y] = rhs(u.node);
            for (Node s : neighbours(u.node)) update_node(s);
        } else {
            G[u.node.x][u.node.y] = INF;
            for (Node s : neighbours(u.node)) update_node(s);
            update_node(u.node);
        }
        // update_map_display();
    }
}

Node choose_min_neighbour() {
    Node minimum = neighbours(start)[0];
    float min_cost = INF;

    for (Node neighbour: neighbours(start)) {
        float n_cost = cost(start, neighbour) + g(neighbour);
        
        if (min_cost > n_cost) {
            minimum = neighbour;
            min_cost = n_cost;
        }
    }

    return minimum;
}

bool initialized = false;

void main1() {
    if (!initialized) {
        path_init();
        last = start;
        compute_shortest_path();
        initialized = true;
    }

    if (start != goal) {
        start = choose_min_neighbour();
        // physically move the robot to this neighbour
        goal_reached = false;
    } else {
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



// TESTING


bool check_free_local(int x, int y) {
    return OBSTACLE_MAP_COPY[x][y] == 0;
}

void populate_map() {
    srand(time(0));
    for (int x = 0; x < MAP_WIDTH; x++)
        for (int y = 0; y < MAP_HEIGHT; y++)
            OBSTACLE_MAP_COPY[x][y] = 0;

    // Large angled wall segments
    int numWalls = 14;
    for (int i = 0; i < numWalls; i++) {
        float cx = rand() % MAP_WIDTH;
        float cy = rand() % MAP_HEIGHT;
        float angle = (rand() % 180) * (float)M_PI / 180.0f;
        float length = 12 + rand() % 8;   // 12-19 cells long
        float thickness = 3 + rand() % 3; // 3-5 cells thick
        float dx = cos(angle), dy = sin(angle);
        float px = -dy, py = dx; // perpendicular direction

        for (float t = -length / 2; t <= length / 2; t += 0.5f) {
            for (float w = -thickness / 2; w <= thickness / 2; w += 0.5f) {
                int x = (int)round(cx + dx * t + px * w);
                int y = (int)round(cy + dy * t + py * w);
                if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
                    OBSTACLE_MAP_COPY[x][y] = 1;
            }
        }
    }

    // Smaller scattered obstacles (1-3 cells each)
    int numSmall = 30;
    for (int i = 0; i < numSmall; i++) {
        int x = rand() % MAP_WIDTH;
        int y = rand() % MAP_HEIGHT;
        OBSTACLE_MAP_COPY[x][y] = 1;
        if (rand() % 2 && x + 1 < MAP_WIDTH) OBSTACLE_MAP_COPY[x + 1][y] = 1;
        if (rand() % 2 && y + 1 < MAP_HEIGHT) OBSTACLE_MAP_COPY[x][y + 1] = 1;
    }

    // Keep a clear pocket around start and goal
    for (int dx = -2; dx <= 2; dx++)
        for (int dy = -2; dy <= 2; dy++) {
            int sx = start.x + dx, sy = start.y + dy;
            if (sx >= 0 && sx < MAP_WIDTH && sy >= 0 && sy < MAP_HEIGHT) OBSTACLE_MAP_COPY[sx][sy] = 0;
            int gx = goal.x + dx, gy = goal.y + dy;
            if (gx >= 0 && gx < MAP_WIDTH && gy >= 0 && gy < MAP_HEIGHT) OBSTACLE_MAP_COPY[gx][gy] = 0;
        }
}

vector<Node> trail;

void update_map_display() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            bool onTrail = false;
            for (auto &t : trail) if (t.x == x && t.y == y) onTrail = true;

            if (x == start.x && y == start.y) cout << 'R';
            else if (x == goal.x && y == goal.y) cout << 'G';
            else if (OBSTACLE_MAP_COPY[x][y]) cout << '#';
            else if (onTrail) cout << '*';
            else cout << '.';
        }
        cout << "\n";
    }
    cout << "---\n";
}

int main() {
    trail.clear();
    populate_map();
    main1();
    trail.push_back(start);

    int guard = 0;
    while (!goal_reached && guard++ < 100) {
        update_map_display();
        main1();
        trail.push_back(start);
        main2();
    }
    update_map_display();
    cout << (goal_reached ? "Reached goal.\n" : "Stuck / no path.\n");
    return 0;
}