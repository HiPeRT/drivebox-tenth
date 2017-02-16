#ifndef PLANNING_H
#define PLANNING_H

#include "common.h"
#include "grid.h"
#include "dinonav.h"

point_t planning(point_t &car_pos, car_t &car, view_t &view, grid_t &grid, path_t &path, segment_t &curve);

int choosegate(grid_t &grid, int px, int py);
segment_t calc_curve(grid_t &grid, int gate_idx, view_t &view, car_t &car);

#endif //PLANNING_H