#ifndef GRID_H
#define GRID_H

#define GRID_MAX_DIM 1000

#include <iostream>
#include <math.h>
#include <stdlib.h>

struct point_t {
    int x, y;
};

class Grid {

public:
    Grid();
    //virtual ~Grid();

    int grid_dim;

    int get(int x, int y);
    bool set(int x, int y, int value);

    void line(int x1, int y1, int x2, int y2);
    bool line_check(int x1, int y1, int x2, int y2);
    void inflate(int x, int y, int val, int n);

    void choosegate(int px, int py, int &to_x, int &to_y);

    int * getGrid() { return grid; }

private:
    int grid[GRID_MAX_DIM*GRID_MAX_DIM];

    point_t gates[64][2];
    int gates_N;

};
#endif //GRID_H