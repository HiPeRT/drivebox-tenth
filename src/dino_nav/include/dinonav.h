#ifndef DINONAV_H
#define DINONAV_H

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "common.h"

#include <dynamic_reconfigure/server.h>
#include <dino_nav/DinonavConfig.h>

struct conf_t {

    float   throttle;
    int     inflation;
    int     grid_dim;
    float   zoom;
    bool    enable;
};

const int EMPTY     = 0;
const int WALL      = 3;
const int INFLATED  = 2;
const int GATE      = 1;
const int PATH      = 4;

struct view_t {

    float x,y;
    float l;
    float cell_l;
};

struct car_t {

    float length;
    float width;
};
 
enum dir_e { LEFT, RIGHT, UP, DOWN };
struct sector_t {
    float l, vel, enter, exit;
    dir_e dir;
};
struct track_t {
    sector_t sects[32];
    int sects_n;
    int cur_sect;
};

struct vels_t {
    ros::Time t;
    float_point_t pos;
    float vel;
};

#include "pathfind.h"

struct dinonav_t {

    conf_t conf;

    view_t view;
    car_t car;

    track_t track;

    grid_t grid;
    path_t path;

    point_t car_pos;
    point_t goal_pos;
    segment_t curve;
    float curve_dst;

    int steer_l;
    float steer, throttle;
    
    float estimated_speed;
    float estimated_acc;
    double yaw = 0;

    float target_acc;
};


void reconf(dino_nav::DinonavConfig &config, uint32_t level);
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg);
void pose_recv(const geometry_msgs::PoseStamped::ConstPtr& msg);
void odom_recv(const nav_msgs::Odometry::ConstPtr& msg);

void init_view(view_t &view, int size);
void init_car(car_t &car, view_t &view, float zoom);
void init(view_t &view, car_t &car, grid_t &grid);

#endif //DINONAV_H
