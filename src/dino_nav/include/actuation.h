#ifndef ACTUATION_H
#define ACTUATION_H

#include "race/drive_param.h"
#include "dinonav.h"

void actuation(dinonav_t &nav, race::drive_param &drive_msg);

float calc_steer(float_point_t &start, view_t &view, car_t &car, grid_t &grid, path_t &path, int &steer_l);
float calc_throttle(view_t &view, car_t &car, track_t &track, segment_t &curve, 
    float curve_dst, float estimated_speed, float estimated_acc, float &target_acc);

#endif //ACTUATION_H