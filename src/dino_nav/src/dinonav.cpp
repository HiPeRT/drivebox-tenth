#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "grid.h"
#include "pathfind.h"
#include "common.h"
#include "viz.h"

#include "dinonav.h"

#include "dino_nav/Stat.h"
#include "race/drive_param.h"
#include "std_msgs/Float32.h"

/////////////////////////////////////////
/* 
    CAR VALUES

    every line is an angle from 0 to 100 

    | steer | throttle  | speed |
    1. steer to use 
    2. throttle to use
    3. speed to be
*/
float car_values[11][3] = {

    { 0,    100,    5.0  }, // 0
    { 60,   100,    2.0  }, // 10
    { 60,    80,    1.5  }, // 20
    { 60,    70,    1.5  }, // 30
    { 100,   80,    1.5  }, // 40
    { 100,   75,    1.3  }, // 50
    { 100,   70,    1.0  }, // 60
    { 100,   65,    1.0  }, // 70
    { 100,   60,    1.0  }, // 80
    {  90,   55,    1.0  }, // 90
    { 100,   50,    1.0  } // 100
};
/////////////////////////////////////////



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
        if(setgrid(grid, grid_x, grid_y, WALL)) {
            grid.points[grid.points_n].x = grid_x;
            grid.points[grid.points_n].y = grid_y;
            grid.points_n++;
        }
        inflate(grid, grid_x, grid_y, INFLATED, nav.inflation);

        //if(i>0 && (last_x != grid_x || last_y != grid_y)) {
        //    grid_line(grid, grid_x, grid_y, last_x, last_y, GATE);
        //}
        angle -= msg->angle_increment;
    }

    for(int i=1; i<grid.points_n; i++) {
        point_t p = grid.points[i];
        point_t prev = grid.points[i-1];
        if(prev.x != p.x || prev.y != p.y) {
            grid_line(grid, p.x, p.y, prev.x, prev.y, GATE);
        }
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

    float prev_ang = 0;
    for(int i=path.size-1; i>=0; i--) { 
        float_point_t fp = fpath[i];
            
        int next_i = i - look_ahead;
        if(next_i <0) next_i = 0;
        float_point_t next = fpath[next_i];

        float sangle_g = prev_ang*180/M_PI/45*100;
        float ang = points_angle(fp.x, fp.y, next.x, next.y) - sangle_g;
        if(ang > 100) ang = 100;
        if(ang < -100)ang = -100;
        path_cost[i] = int(fabs(ang));

        float a = points_angle_rad(fp.x, fp.y, next.x, next.y);
        for(int j=i-1; j>next_i; j--) {
            rotate_point(fpath[j], fp, a - points_angle_rad(fp.x, fp.y, fpath[j].x, fpath[j].y));
        }
        prev_ang = (a + M_PI/2);
    }
}

/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    viz_clear();

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

    float_point_t p; p.x = view.x; p.y = view.y;
    viz_rect(p, view.l, view.l, VIEW_COLOR, 1);
    for(int i=0; i<grid.size; i++) { 
        for(int j=0; j<grid.size; j++) {
            int val = grid.data[i*grid.size +j];
            ALLEGRO_COLOR col;

            if (val == WALL) {
                col = WALL_COLOR;
            } else if(val == INFLATED) {
                col = INFLATED_COLOR;  
            } else if(val == GATE) {
                col = GATE_COLOR;  
            } else {
                continue;
            }
            float_point_t p;
            p.x = view.x + j*view.cell_l;
            p.y = view.y + i*view.cell_l;
            viz_rect(p, view.cell_l, view.cell_l, col, 0);
        }
    }

    car_t car;
    init_car(car, view, nav.zoom);

    int xp = grid.size/2, yp = grid.size - (car.length/10*8)/view.cell_l;
    inflate(grid, xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    int gate_idx = choosegate(grid, xp, yp);
    point_t s = grid.gates[gate_idx].s;
    point_t e = grid.gates[gate_idx].e;
    viz_line(grid2view(s.x, s.y, view), grid2view(e.x, e.y, view), VIEW_COLOR, 2);


    to_x = (s.x + e.x)/2;
    to_y = (s.y + e.y)/2; 
    
    path_t path = pathfinding(grid, view, car, xp, yp, to_x, to_y, nav.stop_cost);
    //get x and y for start and goal from cells position
    float_point_t part = grid2view(xp, yp, view);
    float_point_t goal = grid2view(to_x, to_y, view);

    float path_cost[MAX_ITER];
    calc_path_cost(path_cost, path);

    float car_break_dst = car.length *estimated_speed*2;
    float_point_t break_point = grid2view(to_x, to_y, view);
    float break_point_dst = 10000;
    float break_point_cost = 0;
    for(int i = path.size-1; i>=0; i--) {
        float cost = path_cost[i];
        if(cost > 2 && break_point_dst > 9999){
            break_point = grid2view(path.data[i].x, path.data[i].y, view);
            break_point_dst = point_dst(part, break_point);
            break_point_cost = cost;

            int end = i - 28;
            if(end <0) end = 0;
            for(i=i-1; i>=end; i--) 
                if(path_cost[i] > cost) break_point_cost = path_cost[i];

            break;
        } 
    } 
    printf("cost: %f ", break_point_cost);

    //compute angle for the steer
    static float precedent_steer = 0;
    float ang  = points_angle(part.x, part.y, goal.x, goal.y);
    if(ang >  100) ang=100;
    if(ang < -100) ang=-100;

    //get break point distance
    static float precedent_throttle = 0;
    float throttle = 0;
    if(fabs(ang) > 2) {
        int a = fabs(ang)/10;
        if(a==0) a = 1;

        float delta = estimated_speed - car_values[a][2];
        if(delta < 0) {
            throttle = car_values[a][1];
            if(break_point_cost < car_break_dst && break_point_cost > fabs(ang)) {
                int a = fabs(break_point_cost)/10;
                if(a==0) a = 1;
                float delta = estimated_speed - car_values[a][2];
                if(delta >0)
                    throttle = - delta*20;
            } else {
                ang = car_values[a][0] *ang/fabs(ang);
            }
            printf("apply racing values s: %f t: %f\n", ang, throttle);
        } else {
                throttle = -delta*20;
            printf("to much speed for s: %f, using t: %f\n", ang, throttle);
        }
        
    } else {
        if(break_point_dst > car_break_dst ) {
            printf("open max throttle\n");
            throttle = precedent_throttle + 5;
        } else {
            int a = fabs(break_point_cost)/10;
            if(a==0) a= 1;
            float delta = estimated_speed - car_values[a][2];
            
            printf("must break to: %f\n", car_values[a][2]);
            if(precedent_throttle >0) precedent_throttle = 0;
            throttle = precedent_throttle - delta;
        }
    }
    if(throttle > 100) throttle = 100;
    if(throttle < -100) throttle = -100;
    precedent_throttle = throttle;

    float steer = precedent_steer -  (precedent_steer -ang)/2;
    if(steer > 100)  steer = 100;
    if(steer < -100) steer = -100;

    if(nav.enable) {
        race::drive_param m;

        m.velocity = 10;
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

    viz_flip();
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


