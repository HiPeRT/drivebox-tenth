#ifndef GRID_H
#define GRID_H

#include "dinonav.h"
#include "common.h"

#define GRID_MAX_DIM 1000
#define GRID_MAX_GATES 64

struct grid_t {

    int *data;
    int size;

    point_t gates[GRID_MAX_GATES][2];
    int gates_n;
};


void init_grid(grid_t &grid, int size);

void grid_line(grid_t &grid, int x1, int y1, int x2, int y2, int value);
bool grid_line_control(grid_t &grid, int x1, int y1, int x2, int y2);

bool setgrid(grid_t &grid, int x, int y, int value);
int getgrid(grid_t &grid, int x, int y);

void inflate(grid_t &grid, int x, int y, int val, int n);

void choosegate(grid_t &grid, int px, int py, int &to_x, int &to_y);

float_point_t grid2view(int x, int y, view_t &view);
point_t view2grid(float x, float y, view_t &view);

#endif //GRID_H
