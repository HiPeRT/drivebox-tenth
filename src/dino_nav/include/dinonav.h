#ifndef DINONAV_H
#define DINONAV_H

#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"

#include <dynamic_reconfigure/server.h>
#include <dino_nav/DinonavConfig.h>

struct dinonav_t {

    float   speed;
    int     inflation;
    int     stop_cost;
    int     grid_dim;
    float   zoom;
    bool    enable;
};


void reconf(dino_nav::DinonavConfig &config, uint32_t level);
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg);
void pose_recv(const geometry_msgs::PoseStamped::ConstPtr& msg);

#endif //DINONAV_H
