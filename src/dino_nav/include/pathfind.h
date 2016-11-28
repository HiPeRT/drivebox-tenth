/** Struttura contenente i dati dei nodi utilizzati nel ::pathfinding */
struct nodo {
    int x;
    int y;
    nodo *succ;
    nodo *prec;
};

void pathfinding(int path[], int grid[], int xp, int yp, int &xa, int &ya);

