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

ros::Publisher drive_pub, stat_pub;

dinonav_t nav;

geometry_msgs::Pose pose;
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

void init_car(car_t &car, view_t &view, float zoom) {
    float mul = (view.l/100);
    car.length = (18.0f/zoom)*mul;     
    car.width  = (10.0f/zoom)*mul;
}

void calc_path_cost(float *path_cost, path_t &path) {

    float_point_t fpath[MAX_ITER];    
    for(int i=0; i<path.size; i++) {
        fpath[i].x = path.data[i].x;
        fpath[i].y = path.data[i].y;

        path_cost[i] = 0;
    }
     
    int look_ahead = path.size - path.start;
    int break_anticipe = look_ahead/2;

    for(int i=path.size-1; i>=0; i--) { 
        float_point_t fp = fpath[i];

        float dx = fp.x - path.data[i].x;
        float dy = fp.y - path.data[i].y;
        if(i + break_anticipe < path.size)
            path_cost[i + break_anticipe] = sqrt(dx*dx + dy*dy); 
            
        int next_i = i - look_ahead;
        if(next_i <0) next_i = 0;
        float_point_t next = fpath[next_i];

        float a = points_angle_rad(fp.x, fp.y, next.x, next.y);
        for(int j=i-1; j>next_i; j--) {
            rotate_point(fpath[j], fp, a - points_angle_rad(fp.x, fp.y, fpath[j].x, fpath[j].y));
        }
    }
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
    static int *grid_addr=NULL;
    if(grid_addr == NULL)
        grid_addr = new int[GRID_MAX_DIM*GRID_MAX_DIM];
    grid.data = grid_addr;
    init_grid(grid, nav.grid_dim);

    discretize_laserscan(grid, view, msg);

    car_t car;
    init_car(car, view, nav.zoom);

    int xp = grid.size/2, yp = grid.size - (car.length/10*8)/view.cell_l;
    inflate(grid, xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    choosegate(grid, xp, yp, to_x, to_y);
    
    path_t path = pathfinding(grid, view, car, xp, yp, to_x, to_y, nav.stop_cost);

    float path_cost[MAX_ITER];
    calc_path_cost(path_cost, path);

    float_point_t break_point = grid2view(to_x, to_y, view);
    for(int i = path.size-1; i>=0; i--) {
        float cost = path_cost[i];
        if(cost > 2){
            break_point = grid2view(path.data[i].x, path.data[i].y, view);
            break;
        }
    } 

    //get x and y for start and goal from cells position
    float_point_t part = grid2view(xp, yp, view);
    float_point_t goal = grid2view(to_x, to_y, view);

    //compute angle for the steer
    static float precedent_steer = 0;
    float ang  = points_angle(part.x, part.y, goal.x, goal.y);
    if(ang >  100) ang=100;
    if(ang < -100) ang=-100;
    float steer = precedent_steer -  (precedent_steer -ang)/2;

    //get break point distance
    float break_point_dst = point_dst(part, break_point);
    float car_break_dst = car.length *estimated_speed;
    float min_speed = 1.5 + (break_point_dst - car_break_dst)/(car_break_dst*4); 
    if(min_speed < 1.5)
        min_speed = 1.5;

    printf("min_speed %f  \tbreak_point_dst %f\t car_break %f\n", min_speed, break_point_dst, car_break_dst);

    static float precedent_throttle = 0;
    float delta_speed = min_speed-estimated_speed;
    float throttle = precedent_throttle;
    if(delta_speed > 0)
        throttle = precedent_throttle + delta_speed*delta_speed;
    if(delta_speed < -0.5)
        throttle = precedent_throttle + delta_speed*estimated_speed*2;
    if(precedent_throttle < 0 && delta_speed > 0)
        throttle = 0;

    if(throttle > 100) throttle = 100;
    if(throttle < -100) throttle = -100;
    precedent_throttle = throttle;


    if(nav.enable) {
        race::drive_param m;

        m.velocity = throttle;
        m.angle = steer;
        drive_pub.publish(m);
    }

    //PUB stats for viewer
    dino_nav::Stat stat;
    stat.grid_size = grid.size;
    std::vector<signed char> vgrd(grid.data, grid.data+(grid.size*grid.size));
    stat.grid = vgrd;
    stat.zoom = nav.zoom;

    stat.path_size = path.size;
    stat.path_start = path.start;
    for(int i=0; i<path.size; i++) {     
        dino_nav::Point point;
        point.x = path.data[i].x;
        point.y = path.data[i].y;

        stat.path.push_back(point);
        stat.path_cost.push_back(path_cost[i]);
    }
    stat.throttle = throttle;
    stat.steer = steer;
    stat.speed = estimated_speed;
    stat.pose = pose;
    stat_pub.publish(stat);

    PTIME_END()
    PTIME_STAMP(,DINONAV)
}


void update_speed(geometry_msgs::Point p, ros::Time time) {

    static geometry_msgs::Point old_pose;
    static ros::Time old_time;
    static bool init=false;
    if(!init) {
        old_pose.x = 0;
        old_pose.y = 0;
        old_pose.z = 0;
        init = true;

        old_time = time;
    }

    geometry_msgs::Point pos = p;
    ros::Time t = time;

    double dx = pos.x - old_pose.x;
    double dy = pos.y - old_pose.y;
    double dst = sqrt(dx*dx + dy*dy);
    double dt = (t - old_time).toSec();

    //update speed value
    estimated_speed = dst/dt;
    
    old_pose = pos;
    old_time = t;

    /*
    std_msgs::Float32 m;
    m.data = estimated_speed;
    speed_pub.publish(m);
    */
    
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
}


