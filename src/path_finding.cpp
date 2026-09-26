#include <stdint.h>
#include "path_finding.h"
#include "map.h"
#include "pose.h"
#include <queue>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;
#include "arena_config.h"
#include "debug_print.h"
#include "driving_controller.h"

const int MAP_WIDTH = 97 + 4; // 2 cells at each extrema for walls
const int MAP_HEIGHT = 49 + 4;

int CLEARANCE = 6; // based on size of robot
int PENALTY_WEIGHT = 3;
static Node lastPrintedWaypoint =
{
    -1000,
    -1000
};

static const unsigned long DSTAR_REPAIR_INTERVAL_MS = 150;

static unsigned long lastDstarRepair = 0;

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
float k_m;
vector<Pair> U; // custom priority queue
float RHS[MAP_WIDTH][MAP_HEIGHT]; // next node, "beside"
float G[MAP_WIDTH][MAP_HEIGHT];  // current shortest cost to reach start from goal
vector<Node> changed_cells;
bool goal_reached = false;
bool initialized = false;
const int INF = 65535;


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

float wallPenalty(Node p)
{
    float minDist = (float)CLEARANCE;

    for (int dx = -CLEARANCE; dx <= CLEARANCE; dx++)
    {
        for (int dy = -CLEARANCE; dy <= CLEARANCE; dy++)
        {
            int x = p.x + dx;
            int y = p.y + dy;

            if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT)
            {
                continue;
            }

            if (!check_obstacle(x, y))
            {
                continue;
            }

            float d = heuristic(p,{x, y});
            if (d < minDist)
            {
                minDist = d;
            }
        }
    }

    if (minDist >= CLEARANCE)
    {
        return 0.0f;
    }

    return
        (CLEARANCE - minDist) * PENALTY_WEIGHT;
}

float cost(Node p, Node q) {
    if (!check_free(p.x, p.y) || !check_free(q.x, q.y)) return INF;
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


bool path_init() {
    U.clear(); // reset U
    changed_cells.clear();
    k_m = 0.0f; // distance from start position

    // start = {2, 1};
    // goal = {70,50};
    start.x = world_to_cell_x(pose_get_x_mm());
    start.y = world_to_cell_y(pose_get_y_mm());

    goal.x = world_to_cell_x(arena_get_home_x_mm()); //using actual home coorindates instead of 2500, 2500 which isnt a valid coord
    goal.y = world_to_cell_y(arena_get_home_y_mm());

    last = start;
    goal_reached = false;
    for (int x = 0; x < MAP_WIDTH; x++)
        for (int y = 0; y < MAP_HEIGHT; y++) {
            RHS[x][y] = INF;
            G[x][y] = INF;
        }
    RHS[goal.x][goal.y] = 0;
    Pair goal_pair = {goal, calculate_key(goal)};
    insert(goal_pair, U);
    
    //added in
    initialized = true;
    unsigned long startUs = micros();

    compute_shortest_path();
    unsigned long elapsedUs = micros() - startUs;
    bool routeFound = g(start) < INF;
    debugNav.print("DSTAR_INIT,");
    debugNav.print(start.x);
    debugNav.print(",");
    debugNav.print(start.y);
    debugNav.print(",");
    debugNav.print(goal.x);
    debugNav.print(",");
    debugNav.print(goal.y);
    debugNav.print(",");
    debugNav.print(g(start));
    debugNav.print(",");
    debugNav.print(elapsedUs);
    debugNav.print(",");
    debugNav.println(
        routeFound ? 1 : 0
    );

    return routeFound;
}

void update_node(Node p) {
    if (!(p == goal)) {
        float min_rhs = INF;
        for (Node s : neighbours(p)) {
            float val = cost(p, s) + g(s);
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
        //only repair empy cells every 150ms so it does htem in batches
        if (millis() - lastDstarRepair <
            DSTAR_REPAIR_INTERVAL_MS)
        {
            return;
        }

        lastDstarRepair = millis();
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

void path_notify_cell_changed(
    int x,
    int y)
{
    if (!initialized)
    {
        return;
    }

    Node cell = {x, y};

    for (const Node &existing :
         changed_cells)
    {
        if (existing == cell)
        {
            return;
        }
    }

    changed_cells.push_back(cell);
}


void path_update()
{
    if (!initialized)
    {
        return;
    }

    Node newStart = {world_to_cell_x(pose_get_x_mm()),
                    world_to_cell_y(pose_get_y_mm())};

    bool startChanged = newStart != start;
    
    bool mapChanged = !changed_cells.empty();

    if (startChanged)
    {
        k_m +=
            heuristic(
                start,
                newStart
            );

        start = newStart;
        last = start;
    }

    if (!changed_cells.empty())
    {
        for (Node c :
             changed_cells)
        {
            // Wall penalty means one changed obstacle
            // affects cells around it as well.
            for (int dx = -CLEARANCE;
                 dx <= CLEARANCE;
                 dx++)
            {
                for (int dy = -CLEARANCE;
                     dy <= CLEARANCE;
                     dy++)
                {
                    Node affected = {
                        c.x + dx,
                        c.y + dy
                    };

                    if (affected.x < 0 ||
                        affected.x >= MAP_WIDTH ||
                        affected.y < 0 ||
                        affected.y >= MAP_HEIGHT)
                    {
                        continue;
                    }

                    update_node(
                        affected
                    );
                }
            }
        }

        changed_cells.clear();
    }
    
    bool needsRepair =
        startChanged ||
        mapChanged ||
        !consistent(start);

    if (needsRepair)
    {
        unsigned long startUs = micros();

        // Only stop for a map change.
        // A normal start-cell change should usually be
        // a very cheap incremental D* repair.
        if (mapChanged)
        {
            motor_control_stop();
        }

        compute_shortest_path();

        unsigned long elapsedUs =
            micros() - startUs;

        debugNav.print("DSTAR_REPLAN,");
        debugNav.print(start.x);
        debugNav.print(",");
        debugNav.print(start.y);
        debugNav.print(",");
        debugNav.print(elapsedUs);
        debugNav.print(",");
        debugNav.println(g(start));
    }
    goal_reached =
        start == goal;
}


static bool bestNeighbourFrom(Node from, Node &best)
{
    float bestCost = INF;
    bool found = false;

    for (Node neighbour :
         neighbours(from))
    {
        float candidate = cost(from,neighbour) + g(neighbour);

        if (candidate < bestCost)
        {
            bestCost = candidate;
            best = neighbour;
            found = true;
        }
    }

    return found &&
           bestCost < INF;
}


bool path_get_next_waypoint(float &x_mm, float &y_mm)
{
    if (!initialized)
    {
        return false;
    }

    path_update();
    if (start == goal ||
        g(start) >= INF)
    {
        return false;
    }

    Node cursor = start;
    Node waypoint = start;

    int firstDx = 0;
    int firstDy = 0;

    // Look several cells ahead, but only along
    // the same straight D* segment. This avoids
    // commanding a 50 mm waypoint every time.
    for (int i = 0;
         i < 4;
         i++)
    {
        Node next;

        if (!bestNeighbourFrom(
                cursor,
                next))
        {
            break;
        }

        int dx = next.x - cursor.x;

        int dy = next.y - cursor.y;

        if (i == 0)
        {
            firstDx = dx;
            firstDy = dy;
        }
        else if (
            dx != firstDx || dy != firstDy)
        {
            // Direction changed: stop before
            // cutting across a corner.
            break;
        }

        waypoint = next;
        cursor = next;

        if (cursor == goal)
        {
            break;
        }
    }

    if (waypoint == start)
    {
        return false;
    }

    x_mm = cell_to_world_x(waypoint.x);

    y_mm = cell_to_world_y(waypoint.y);

    if (waypoint != lastPrintedWaypoint)
    {
        lastPrintedWaypoint =
            waypoint;

        debugNav.print("DSTAR_WAYPOINT,");
        debugNav.print(waypoint.x);
        debugNav.print(",");
        debugNav.print(waypoint.y);
        debugNav.print(",");
        debugNav.print(x_mm);
        debugNav.print(",");
        debugNav.println(y_mm);
    }
    return true;
}


void path_reset()
{
    initialized = false;
    goal_reached = false;
    changed_cells.clear();
    U.clear();
}