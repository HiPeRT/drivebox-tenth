#include "Grid.h"

Grid::Grid() {

    grid_dim = 100;
}


void Grid::init() {
    
    //init grid
    for(int i=0; i<grid_dim*grid_dim; i++)
        grid[i] = 0;
    
    gates_N = 0;
}

bool Grid::set(int x, int y, int value) {
    int pos = y*grid_dim + x;

    if(x<0 || x >= grid_dim || y<0 || y >= grid_dim)
        return false;
    grid[pos] = value;
    return true;
}

int Grid::get(int x, int y) {
    int pos = y*grid_dim + x;

    if(x<0 || x >= grid_dim || y<0 || y >= grid_dim)
        return -1;
    return grid[pos];
}

/**
    geterate a line in a matrix from point 1 to 2
*/
void Grid::line(int x1, int y1, int x2, int y2) {
    //gate search
    int startGx = -1, startGy = -1;
    int endGx = -1,   endGy = -1;

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++) {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        if(get(xx, yy) == 0) {
            set(xx, yy, 3);
            if(startGx == -1) {
                startGx = xx; startGy = yy; 
            } else {
                endGx = xx; endGy = yy;
            }
        }
    }

    if(startGx != -1 && endGx != -1) {
        point_t *limit = gates[gates_N];
        limit[0].x = startGx;
        limit[0].y = startGy;
        limit[1].x = endGx;
        limit[1].y = endGy;
        gates_N++;
        //printf("start: %d %d, end: %d %d\n", startGx, startGy, endGx, endGy);
    }
}

/**
    geterate a line in a matrix from point 1 to 2
*/
bool Grid::line_check(int x1, int y1, int x2, int y2) {

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++) {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        int val =  get(xx, yy);
        int val1 = get(xx + 1, yy);
        int val2 = get(xx - 1, yy);
        if(val == 1 || val == 2 || val1 == 1 || val1 == 2 || val2 == 1 || val2 == 2)
            return false;
    }
    return true;
}


/**
    inflate a point with the given spread
          X X X
    X ->  X X X   example with n = 1
          X X X
*/
void Grid::inflate(int x, int y, int val, int n) {
    if(n == 0)
        return;

    if(get(x, y) != 1)
        set(x, y, val);

    inflate(x-1, y-1, val, n -1);
    inflate(x-1, y, val, n -1);
    inflate(x-1, y+1, val, n -1);
    inflate(x+1, y-1, val, n -1);
    inflate(x+1, y, val, n -1);
    inflate(x+1, y+1, val, n -1);
    inflate(x, y-1, val, n -1);
    inflate(x, y+1, val, n -1);
}

void Grid::choosegate(int px, int py, int &to_x, int &to_y) {

    float dst = -1; //min dst
    int gt = -1;

    for(int i=0; i<gates_N; i++) {
        for(int j=0; j<2; j++) {
            
            point_t p = gates[i][j];
            float dx = px - p.x, dy = py - p.y; 
            float d = sqrt(dx*dx + dy*dy); 
            if(d > dst) {
                dst = d;
                gt = i;
            }
        }
    }

    if(gt != -1) {
        point_t s = gates[gt][0];
        point_t e = gates[gt][1];

        to_x = (s.x + e.x)/2;
        to_y = (s.y + e.y)/2;
    } 
}