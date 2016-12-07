#include "Grid.h"

/** Struttura contenente i dati dei nodi utilizzati nel ::pathfinding */
struct nodo {
    int x;
    int y;
    nodo *succ;
    nodo *prec;
};


const int MAX_ITER = 500;

int pathfinding(Grid &grid_, int xp, int yp, int &xa, int &ya, point_t calc_path[]);

