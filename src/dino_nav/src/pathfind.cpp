//
// Created by cecco on 24/11/16.
//
#include <iostream>
#include <math.h>
#include <stdio.h>

#include "dinonav.h"
#include "pathfind.h"

extern int stop_cost;
extern int grid_dim;

static void eject(nodo * &testa, nodo* &coda, int &x, int &y)
{
    x = testa->x;
    y = testa->y;

    nodo *tmp = testa->succ;
    delete [] testa;
    if (coda == testa)
        coda = testa = tmp;
    else
        testa = tmp;
}

static void inject(nodo * &testa, nodo* &coda, int x, int y)
{
    nodo *n = new nodo;

    n->x = x;
    n->y = y;
    if (testa != NULL) {
        n->prec = coda;
        n->succ = NULL;
        coda->succ = n;
        coda = n;
    } else {
        testa = coda = n;
        n->prec = NULL;
        n->succ = NULL;
    }



}

int gridval(int path[], int x, int y) {
    int val = getgrid(path, x, y);
    if(val <= 0)
        return 999999; //as infinite

    return val;
}

int bigger_x, bigger_y, bigger_value;

bool gridcheck(int path[], int grid[], int ox, int oy, int x, int y) {

    int pos = y*grid_dim + x;
    if(x<0 || pos <0 || pos >= grid_dim*grid_dim)
        return false;

    if (grid[pos] == 0 && path[pos] == 0) {
        int val = path[oy*grid_dim + ox] +1;
        path[y*grid_dim + x] = val;

        if(val > bigger_value) {
            bigger_value = val;
            bigger_x = x;
            bigger_y = y;
        }
        return true;
    }
    return false;
}



void pathfinding(int path[], int grid[], int xp, int yp, int &xa, int &ya)
{

    //Algoritmo BFS:
    nodo *testa = NULL;
    nodo *coda = NULL;

    inject(testa, coda, xp, yp);

    setgrid(path, xp, yp, 1);    //partenza
    bigger_x = xp;
    bigger_y = yp;
    bigger_value = 1;

    while(testa != NULL) {

        int x, y;
        eject(testa, coda, x, y);

        if(gridcheck(path, grid, x, y, x+1, y))
            inject(testa, coda, x+1, y);

        if(gridcheck(path, grid, x, y, x-1, y))
            inject(testa, coda, x-1, y);

        if(gridcheck(path, grid, x, y, x, y-1))
            inject(testa, coda, x, y-1);

        if(gridcheck(path, grid, x, y, x, y+1))
            inject(testa, coda, x, y+1);
    }

    //find path
    int x = bigger_x, y = bigger_y;
    while(x != xp || y != yp) {

        int a[8];
        a[0] = gridval(path, x+1, y-1);
        a[1] = gridval(path, x+1, y);
        a[2] = gridval(path, x+1, y+1);
        a[3] = gridval(path, x-1, y-1);
        a[4] = gridval(path, x-1, y);
        a[5] = gridval(path, x-1, y+1);
        a[6] = gridval(path, x,   y-1);
        a[7] = gridval(path, x,   y+1);

        int min_id = 0;
        int min_val = a[0];
        for(int i=1; i<8; i++) {
            if (a[i] < min_val) {
                min_val = a[i];
                min_id = i;
            }
        }

        switch (min_id) {
            case 0: x += 1; y -= 1; break;
            case 1: x += 1; y;      break;
            case 2: x += 1; y += 1; break;
            case 3: x -= 1; y -= 1; break;
            case 4: x -= 1; y;      break;
            case 5: x -= 1; y += 1; break;
            case 6: x;      y -= 1; break;
            case 7: x;      y += 1; break;
        }
        setgrid(grid, x, y, 10);

        if(min_val <= stop_cost) {
            xa = x;
            ya = y;
            return;
        }
    }

}