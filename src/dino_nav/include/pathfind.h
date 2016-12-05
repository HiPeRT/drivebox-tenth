/** Struttura contenente i dati dei nodi utilizzati nel ::pathfinding */
struct nodo {
    int x;
    int y;
    nodo *succ;
    nodo *prec;
};

struct point_t {
    int x, y;
};

const int MAX_ITER = 500;

int pathfinding(int path[], int grid[], int xp, int yp, int &xa, int &ya, point_t calc_path[]);

