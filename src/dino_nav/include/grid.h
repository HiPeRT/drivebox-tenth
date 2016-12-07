#ifndef GRID_H
#define GRID_H

#define GRID_MAX_DIM 1000
#define GRID_MAX_GATES 256

struct point_t {
    int x, y;
};

struct grid_t {

    int data[GRID_MAX_DIM*GRID_MAX_DIM];
    int size;

    point_t gates[GRID_MAX_GATES][2];
    int gates_n;
};


void init_grid(grid_t &grid, int size);

void grid_line(grid_t &grid, int x1, int y1, int x2, int y2);
bool grid_line_control(grid_t &grid, int x1, int y1, int x2, int y2);

bool setgrid(grid_t &grid, int x, int y, int value);
int getgrid(grid_t &grid, int x, int y);

void inflate(grid_t &grid, int x, int y, int val, int n);

void choosegate(grid_t &grid, int px, int py, int &to_x, int &to_y);

#endif //GRID_H
