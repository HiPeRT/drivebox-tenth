//
// Created by cecco on 24/11/16.
//
#include <iostream>
#include <math.h>
#include <stdio.h>

#include "pathfind.h"
#include "dinonav.h"

static void eject(nodo * &testa, nodo* &coda, int &x, int &y) {
    x = testa->x;
    y = testa->y;

    nodo *tmp = testa->succ;
    delete [] testa;
    if (coda == testa)
        coda = testa = tmp;
    else
        testa = tmp;
}

static void inject(nodo * &testa, nodo* &coda, int x, int y) {
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

int gridval(grid_t &path, int x, int y) {
    int val = getgrid(path, x, y);
    if(val <= 0)
        return 999999; //as infinite

    return val;
}

int bigger_x, bigger_y, bigger_value;

bool gridcheck(grid_t &path, grid_t &grid, int ox, int oy, int x, int y) {

    int pos = y*grid.size + x;
    if(x<0 || x >= grid.size || y<0 || y >= grid.size)
        return false;

    if (grid.data[pos] == 0 && path.data[pos] == 0) {
        int val = path.data[oy*grid.size + ox] +1;
        path.data[y*grid.size + x] = val;

        if(val > bigger_value) {
            bigger_value = val;
            bigger_x = x;
            bigger_y = y;
        }
        return true;
    }
    return false;
}


void init_path(path_t &path) {
    path.size = 0;
    path.start = 0;
}

path_t pathfinding(grid_t &grid, view_t &view, car_t &car, int xp, int yp, int &xa, int &ya, int stop_cost)
{
    //path to calc
    path_t path;
    init_path(path);

    //path grid
    grid_t path_grid;
    static int *path_addr=NULL;
    if(path_addr == NULL)
        path_addr = new int[GRID_MAX_DIM*GRID_MAX_DIM];
    path_grid.data = path_addr;
    init_grid(path_grid, grid.size);

    //Algoritmo BFS:
    nodo *testa = NULL;
    nodo *coda = NULL;

    inject(testa, coda, xp, yp);

    setgrid(path_grid, xp, yp, 1);    //partenza
    bigger_x = xp;
    bigger_y = yp;
    bigger_value = 1;

    while(testa != NULL) {

        int x, y;
        eject(testa, coda, x, y);

        if(gridcheck(path_grid, grid, x, y, x+1, y))
            inject(testa, coda, x+1, y);

        if(gridcheck(path_grid, grid, x, y, x-1, y))
            inject(testa, coda, x-1, y);

        if(gridcheck(path_grid, grid, x, y, x, y-1))
            inject(testa, coda, x, y-1);

        if(gridcheck(path_grid, grid, x, y, x, y+1))
            inject(testa, coda, x, y+1);
    }

    //find path
    int x,y;
    if(xa == -1) {
        x = bigger_x;
        y = bigger_y;
    } else {
        x = xa;
        y = ya;
    }
    
    int iter = 0;

    while(iter <MAX_ITER && (x != xp || y != yp) ) {
        iter++;

        int a[8];
        a[0] = gridval(path_grid, x+1, y-1);
        a[1] = gridval(path_grid, x+1, y);
        a[2] = gridval(path_grid, x+1, y+1);
        a[3] = gridval(path_grid, x-1, y-1);
        a[4] = gridval(path_grid, x-1, y);
        a[5] = gridval(path_grid, x-1, y+1);
        a[6] = gridval(path_grid, x,   y-1);
        a[7] = gridval(path_grid, x,   y+1);

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

        if(path.start == 0 && min_val <= stop_cost)
            path.start = path.size;

        path.data[path.size].x = x;
        path.data[path.size].y = y;
        path.size++;
    }

    if(iter >= MAX_ITER && xa != -1 && ya != -1) {
        //recalc
        xa = -1;
        ya = -1;
        ROS_WARN("GATE PATH INVALID, recalc with standard method");
        //TODO: deal with stack overflow!!
        return pathfinding(grid, view, car, xp, yp, xa, ya, stop_cost);
    }

    //expand path
    for(int i=path.size-1; i>=1; i--) {
        point_t p, p1;
        p.x = path.data[i].x;
        p.y = path.data[i].y;
        p1.x = path.data[i-1].x;
        p1.y = path.data[i-1].y;

        int v = i - (path.size - path.start);
        if(v < 0) v=0;
        float_point_t goal = grid2view(path.data[v].x, path.data[v].y, view);

        //al_draw_filled_rectangle(view.x + view.cell_l * p.x, view.y + view.cell_l * p.y, view.x + view.cell_l * (p.x + 1),
        //                         view.y + view.cell_l * (p.y + 1), PATH_GRID_COLOR);
                       
        float_point_t fp = grid2view(p.x, p.y, view);
        float_point_t fp1 = grid2view(p1.x, p1.y, view);
        float ang  = points_angle_rad(fp.x, fp.y, fp1.x, fp1.y) - M_PI;

        //al_draw_line(fp.x, fp.y, goal.x, goal.y, PATH_COLOR, 1);
        
        
        float cw = car.width;

        int lv, rv;
        float_point_t left, right;
        for(int j=0; j<cw/view.cell_l; j++) {
            left.x  = fp.x + cos(ang + M_PI/2)*cw;   left.y  = fp.y + sin(ang + M_PI/2)*cw;
            right.x = fp.x + cos(ang - M_PI/2)*cw;   right.y = fp.y + sin(ang - M_PI/2)*cw;
        
            point_t lg = view2grid(left.x, left.y, view);
            point_t rg = view2grid(right.x, right.y, view);
            lv = getgrid(grid, lg.x, lg.y);
            rv = getgrid(grid, rg.x, rg.y);

            if(lv != EMPTY && rv == EMPTY) {
                fp.x += cos(ang - M_PI/2)*view.cell_l;
                fp.y += sin(ang - M_PI/2)*view.cell_l;
            
            } else if (lv == EMPTY && rv != EMPTY) {
                fp.x += cos(ang + M_PI/2)*view.cell_l;
                fp.y += sin(ang + M_PI/2)*view.cell_l; 
            
            } 
        }

        path.data[i] = view2grid(fp.x, fp.y, view);
    }


    //shortcuts path
    for(int i=path.start; i<path.size; i++) {
        int x = path.data[i].x;
        int y = path.data[i].y;

        if(grid_line_control(grid, xp, yp, x, y)) {
            path.start = i;
            break;
        }
    }

    //printf("choosen p = %d - %d\n", n_p, path_l);
    xa = path.data[path.start].x;
    ya = path.data[path.start].y;
    return path;
}