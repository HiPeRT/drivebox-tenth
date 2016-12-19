#ifndef PATHFIND_H
#define PATHFIND_H

#include "grid.h"

/** Struttura contenente i dati dei nodi utilizzati nel ::pathfinding */
struct node_t {
    float_point_t pos;
    float angle;
    float cost;

    //this are for node identification
    node_t *parent;
    int steer;
};

const int MAX_ITER = 500;

struct path_t {

    node_t data[MAX_ITER];
    int size;

    int start;
};

path_t pathfinding(grid_t &grid, view_t &view, float_point_t &s, float_point_t &e);

#endif //PATHFIND_H