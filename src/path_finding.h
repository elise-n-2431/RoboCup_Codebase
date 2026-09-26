//
// Created by elise on 25/09/2026.
//

#ifndef PATH_FINDING_H
#define PATH_FINDING_H

struct Node { // represent possible positions
    int x;
    int y;

    bool operator==(const Node &other) const {
        return (x == other.x) && (y == other.y);
    }

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


void path_init();
void path_update();

#endif