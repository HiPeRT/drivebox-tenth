#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "grid.h"
#include "pathfind.h"
#include "common.h"

#include "dinonav.h"

#include "dino_nav/Stat.h"
#include "race/drive_param.h"
#include "std_msgs/Float32.h"

extern ros::Publisher drive_pub, map_pub, speed_pub;

dinonav_t nav;
float estimated_speed;


void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  nav.speed = config.speed;
  nav.inflation = config.inflation;
  nav.stop_cost = config.stop_cost;
  nav.grid_dim = config.grid_dim;
  nav.zoom = config.zoom;
  nav.enable = config.enable;
}


void discretize_laserscan(grid_t &grid, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg) {

    int last_x, last_y;

    float maxd = nav.zoom;
    int quad_l = maxd*2;
    int size = msg->ranges.size();
    double angle = msg->angle_max + M_PI*3/2;

    for(int i=0; i<size; i++) {
        float r = msg->ranges[i];

        //quad_l : view_l = r : view_r
        //coodianates of the sigle ray
        float view_r = r*view.l/quad_l;
        float x = view.l/2 + cos(angle) * view_r;
        float y = view.l + sin(angle) * view_r;

        //coordinates of the corrispondent cell
        int grid_x = x / view.cell_l;
        int grid_y = y / view.cell_l;
        setgrid(grid, grid_x, grid_y, WALL);
        inflate(grid, grid_x, grid_y, INFLATED, nav.inflation);

        if(i>0 && (last_x != grid_x || last_y != grid_y)) {
            grid_line(grid, grid_x, grid_y, last_x, last_y, GATE);
        }
        last_x = grid_x;
        last_y = grid_y;


        angle -= msg->angle_increment;
    }
}

void init_view(view_t &view, int size) {
    view.x = 10; view.y = 10;
    view.l = 512;
    view.cell_l = view.l / float(size);
}

void init_car(car_t &car, view_t &view) {
    car.length = (view.l/100)*4 / view.cell_l;
}

/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ROS_INFO("Scan recived: [%f]", msg->scan_time);
    PTIME_INIT()
    PTIME_START()

    view_t view;
    init_view(view, nav.grid_dim);

    grid_t grid;
    init_grid(grid, nav.grid_dim);

    discretize_laserscan(grid, view, msg);

    car_t car;
    init_car(car, view);

    int xp = grid.size/2, yp = grid.size - car.length;
    inflate(grid, xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    choosegate(grid, xp, yp, to_x, to_y);
    
    path_t path = pathfinding(grid, xp, yp, to_x, to_y, nav.stop_cost);

    //get x and y for start and goal from cells position
    float x_part = view.x + xp*view.cell_l + view.cell_l/2,   y_part = view.y + yp*view.cell_l + view.cell_l/2;
    float x_goal = view.x + to_x*view.cell_l + view.cell_l/2, y_goal = view.y + to_y*view.cell_l + view.cell_l/2;

    //compute angle for the steer
    float ang  = points_angle(x_part, y_part, x_goal, y_goal);
    
    //compute angle ahead for the speed modulation
    int ah_idx = path.start -4;
    if(ah_idx < 0) ah_idx = 0;
    point_t ahead_p = path.data[ah_idx];
    x_goal = view.x + ahead_p.x*view.cell_l + view.cell_l/2;
    y_goal = view.y + ahead_p.y*view.cell_l + view.cell_l/2;
    float ang2 = points_angle(x_part, y_part, x_goal, y_goal);

    if(nav.enable) {
        race::drive_param m;
        /*
        if(fabs(ang2 - ang) > 30 && estimated_speed > 1.5) {
            m.velocity = -10;
            printf("BREAK!!!\n");
        } else {
	    if(fabs(ang2 - ang) < 10 && estimated_speed < 3)
        	m.velocity = nav.speed*2;
	    else
		m.velocity = nav.speed;
        }
        */

        m.velocity = nav.speed;
        m.angle = ang;
        if(m.angle >  100) m.angle=100;
        if(m.angle < -100) m.angle=-100;

        drive_pub.publish(m);
    }

    //PUB stats for viewer
    dino_nav::Stat stat;
    stat.grid_size = grid.size;
    std::vector<signed char> vgrd(grid.data, grid.data+(grid.size*grid.size));
    stat.grid = vgrd;

    stat.path_size = path.size;
    stat.path_start = path.start;
    for(int i=0; i<path.size; i++) {     
        dino_nav::Point point;
        point.x = path.data[i].x;
        point.y = path.data[i].y;

        stat.path.push_back(point);
    }
    map_pub.publish(stat);

    PTIME_END()
    PTIME_STAMP(,DINONAV)
}


/**
    PoseStamped callback
*/
void pose_recv(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    static geometry_msgs::Point old_pose;
    static ros::Time old_time;
    static bool init=false;
    if(!init) {
        old_pose.x = 0;
        old_pose.y = 0;
        old_pose.z = 0;
        init = true;

        old_time = msg->header.stamp;
    }

    geometry_msgs::Point pos = msg->pose.position;
    ros::Time t = msg->header.stamp;

    double dx = pos.x - old_pose.x;
    double dy = pos.y - old_pose.y;
    double dst = sqrt(dx*dx + dy*dy);
    double dt = (t - old_time).toSec();

    //update speed value
    estimated_speed = dst/dt;
    
    old_pose = pos;
    old_time = t;

    std_msgs::Float32 m;
    m.data = estimated_speed;
    speed_pub.publish(m);
}


