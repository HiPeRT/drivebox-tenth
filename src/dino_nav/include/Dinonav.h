#ifndef DINONAV_H
#define DINONAV_H

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <dynamic_reconfigure/server.h>
#include <dino_nav/DinonavConfig.h>

#include "ros/ros.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"

#include "Grid.h"
#include "pathfind.h"
#include "common.h"

void reconf(dino_nav::DinonavConfig &config, uint32_t level);
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg);
void pose_recv(const geometry_msgs::PoseStamped::ConstPtr& msg);


#endif //DINONAV_H