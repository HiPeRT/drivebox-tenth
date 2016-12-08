#ifndef PATHFIND_H
#define PATHFIND_H

#include "grid.h"

/** Struttura contenente i dati dei nodi utilizzati nel ::pathfinding */
struct nodo {
    int x;
    int y;
    nodo *succ;
    nodo *prec;
};

const int MAX_ITER = 500;

struct path_t {

    point_t data[MAX_ITER];
    int size;

    int start;
};

path_t pathfinding(grid_t &grid, int xp, int yp, int &xa, int &ya, int stop_cost);

#endif //PATHFIND_H