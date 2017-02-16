#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "common.h"
#include "grid.h"
#include "dinonav.h"

point_t perception(grid_t &grid, car_t &car, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg);

void discretize_laserscan(grid_t &grid, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg);

#endif //PERCEPTION_H