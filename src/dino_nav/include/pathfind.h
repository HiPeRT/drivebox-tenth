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

int pathfinding(grid_t &path, grid_t &grid, int xp, int yp, int &xa, int &ya, point_t calc_path[], int stop_cost);

#endif //PATHFIND_H