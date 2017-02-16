#ifndef ACTUATION_H
#define ACTUATION_H

#include "race/drive_param.h"

int actuation(point_t &part, point_t &goal, view_t &view, car_t &car, grid_t &grid, path_t &path, segment_t &curve, race::drive_param &drive_msg);

float calc_steer(float_point_t &start, view_t &view, car_t &car, grid_t &grid, path_t &path, int &steer_l);
float calc_throttle(segment_t &curve, float curve_dst, view_t &view, car_t &car, float &target_acc);

#endif //ACTUATION_H