#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "grid.h"
#include "pathfind.h"
#include "common.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

#include "dinonav.h"
#include "perception.h"
#include "planning.h"
#include "actuation.h"


#include "dino_nav/Stat.h"
#include "race/drive_param.h"
#include "std_msgs/Float32.h"

ros::Publisher drive_pub, stat_pub;

dinonav_t nav;
track_t track;

geometry_msgs::Pose pose;
float estimated_speed;
float estimated_acc;
double yaw = 0;

void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  nav.speed = config.speed;
  nav.inflation = config.inflation;
  nav.stop_cost = config.stop_cost;
  nav.grid_dim = config.grid_dim;
  nav.zoom = config.zoom;
  nav.enable = config.enable;
}

void init_view(view_t &view, int size) {
    view.x = 10; view.y = 10;
    view.l = 512;
    view.cell_l = view.l / float(size);
}

void init_car(car_t &car, view_t &view, float zoom) {
    float mul = (view.l/100);
    car.length = (18.0f/zoom)*mul;     
    car.width  = (10.0f/zoom)*mul;
}

void init(view_t &view, car_t &car, grid_t &grid) {

    init_view(view, nav.grid_dim);
    init_car(car, view, nav.zoom);

    static int *grid_addr=NULL;
    if(grid_addr == NULL)
        grid_addr = new int[GRID_MAX_DIM*GRID_MAX_DIM];
    grid.data = grid_addr;
    init_grid(grid, nav.grid_dim);
}

/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    viz_clear();

    //ROS_INFO("Scan recived: [%f]", msg->scan_time);
    ros::WallTime time_debug = ros::WallTime::now();

    view_t view;
    grid_t grid;
    car_t car;
    init(view, car, grid);
    
    point_t car_pos = perception(grid, car, view, msg);

    draw_car(view, car);
    draw_grid(grid, view);   

    path_t path;
    segment_t curve;
    point_t goal_pos = planning(car_pos, car, view, grid, path, curve);
    
    draw_yaw(yaw, view); 
    draw_track(track, view);
    draw_path(path);

    race::drive_param drive_msg;
    int steer_l = actuation(car_pos, goal_pos, view, car, grid, path, curve, drive_msg);
    if(nav.enable) 
        drive_pub.publish(drive_msg);

    double time = (ros::WallTime::now() - time_debug).toSec();
    #ifndef TIME_PROFILER
    if(time >= 1/40.0f)
        printf("iter time exceded: %lf\n", time);
    #else
        double start = time_debug.toSec();
        double end = start + time;
        printf("DINONAV %lf %lf %lf\n", start, end, time);
    #endif


    //PUB stats for viewer
    dino_nav::Stat stat;
    stat.car_w = car.width;
    stat.car_l = car.length;

    stat.grid_size = grid.size;
    std::vector<signed char> vgrd(grid.data, grid.data+(grid.size*grid.size));
    stat.grid = vgrd;
    stat.zoom = nav.zoom;

    stat.steer_l = steer_l;
    stat.throttle = drive_msg.velocity;
    stat.steer = drive_msg.angle;
    stat.speed = estimated_speed;
    stat.acc = estimated_acc;
    stat.pose = pose;
    stat_pub.publish(stat);

    viz_flip();
}

void update_speed(geometry_msgs::Point p, ros::Time time) {

    static bool init=false;
    const int VELS_DIM = 10;
    static vels_t vels[VELS_DIM];
    static int now = 0;

    if(!init) {
        for(int i=0; i< VELS_DIM; i++) {
            vels[i].t = time;
            vels[i].pos.x = 0;
            vels[i].pos.y = 0; 
	    vels[i].vel = 0;
        }
        init = true;
    }
    vels[now].pos.x = p.x;
    vels[now].pos.y = p.y;
    vels[now].t = time;

    estimated_speed = 0;
    for(int i=1; i<VELS_DIM/2; i++) {
        int idx = (now+i) % VELS_DIM;

        double dx = vels[now].pos.x - vels[idx].pos.x;
        double dy = vels[now].pos.y - vels[idx].pos.y;
        double dst = sqrt(dx*dx + dy*dy);
        double dt = (vels[now].t - vels[idx].t).toSec();

        //sum for mean
        estimated_speed += dst/dt;
    }
    estimated_speed /= (VELS_DIM/2 -1);
    estimated_acc = (estimated_speed - vels[(now+1)%VELS_DIM].vel) / 
      (vels[now].t - vels[(now+1)%VELS_DIM].t).toSec(); 

    vels[now].vel = estimated_speed;
    now = (now+1) % VELS_DIM;
}

/**
    PoseStamped callback
*/
void pose_recv(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    pose = msg->pose;
    update_speed(msg->pose.position, msg->header.stamp);
}

/**
    Odometry callback
*/
void odom_recv(const nav_msgs::Odometry::ConstPtr& msg) {

    pose = msg->pose.pose;
    update_speed(msg->pose.pose.position, msg->header.stamp);

    tf::Quaternion q(   pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}


