#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
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
track_t track;

geometry_msgs::Pose pose;
float estimated_speed;
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


void discretize_laserscan(grid_t &grid, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg) {

    float maxd = nav.zoom;
    int quad_l = maxd*2;
    int size = msg->ranges.size();
    double angle = msg->angle_max + M_PI*3/2;

    for(int i=0; i<size; i++) {
        float r = msg->ranges[i];
        
        if(i==size/2)
            grid.middle_id = grid.points_n;

        //quad_l : view_l = r : view_r
        //coodianates of the sigle ray
        float view_r = r*view.l/quad_l;
        float x = view.l/2 + cos(angle) * view_r;
        float y = view.l + sin(angle) * view_r;

        //coordinates of the corrispondent cell
        int grid_x = x / view.cell_l;
        int grid_y = y / view.cell_l;
        if(setgrid(grid, grid_x, grid_y, WALL)) {

            int n = grid.points_n;
            if( n == 0 || ( n>0                            &&
                            grid.points[n-1].x != grid_x   ||
                            grid.points[n-1].y != grid_y       )) {

                grid.points[n].x = grid_x;
                grid.points[n].y = grid_y;
                grid.points_n++;
            }
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


void draw_track(track_t &track, view_t &view) { 
    float_point_t pos;
    pos.x = view.x + view.l + 100; pos.y = view.y + 300;
    float s_ang = M_PI/2 + M_PI;
    float dim = 150;

    float_point_t p = pos;
    for(int i=0; i<track.sects_n; i++) {
        sector_t s = track.sects[i];
        float_point_t next;

        next.x = p.x + cos(s_ang)*s.l*dim;
        next.y = p.y + sin(s_ang)*s.l*dim;
        
        viz_line(p, next, VIEW_COLOR, 1 + (track.cur_sect == i)*4);
        p = next;

        if(s.dir == LEFT)
            s_ang -= M_PI/2;
        else
            s_ang += M_PI/2; 
    }

}

void draw_grid(grid_t &grid, view_t &view) {
    
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

    p.x = view.x + grid.points[grid.middle_id].x*view.cell_l;
    p.y = view.y + grid.points[grid.middle_id].y*view.cell_l;
    viz_rect(p, view.cell_l, view.cell_l, PATH_COLOR, 0);
}

void draw_signal(float_point_t center, float r, dir_e d) {

    viz_circle(center, r, RGBA(1,1,1,1), 0);
    viz_circle(center, r, RGBA(1,0,0,1), r/5);
    float_point_t i, e;
    i.x = center.x - r/1.5;
    e.x = center.x + r/1.5;
    i.y = e.y = center.y; 
    viz_line(i, e, RGBA(0,0,0,1), r/10);
    
    float_point_t a, b;
    if(d == LEFT) {
        a.x = i.x + r/3;
        b.x = i.x + r/3;
    } else {
        i.x = e.x;
        a.x = i.x - r/3;
        b.x = i.x - r/3;
    }
    
    a.y = i.y - r/3;
    b.y = i.y + r/3;
    viz_triangle(i, a, b, RGBA(0,0,0,1), 0);
}

point_t calc_curve(grid_t &grid, int gate_idx, view_t &view) {
    
    point_t curve;
    curve.x = -1;
    curve.y = -1;

    point_t g1 = grid.gates[gate_idx].s;
    point_t g2 = grid.gates[gate_idx].e;

    point_t internal, external;
    int sign;

    int point1=-1, point2=-1;
    for(int i=0; i<grid.points_n; i++) {
        if( (grid.points[i].x == g1.x && grid.points[i].y == g1.y)     ||
            (grid.points[i].x == g2.x && grid.points[i].y == g2.y)    ) {
            
            if(point1 <0)
                point1 = i;
            else if(point2 <0) {
                point2 = i;
                break;
            }
        }
    }

/*
    int point_idx;
    if(point1 < grid.middle_id && point2 < grid.middle_id) {
        sign = -1;
        internal = grid.points[point1];
        external = grid.points[point2];
        point_idx = point1;
        viz_circle(grid2view(internal.x, internal.y, view), 2, PATH_COLOR, 1);

    } else if (point1 >= grid.middle_id && point2 >= grid.middle_id) {
        sign = +1;
        internal = grid.points[point2];
        external = grid.points[point1];
        point_idx = point2;
        viz_circle(grid2view(internal.x, internal.y, view), 2, PATH_COLOR, 1);

    } else {
        return;
    }*/

    internal = grid.points[point1];
    external = grid.points[point2];
    float_point_t i = grid2view(internal.x, internal.y, view);
    float_point_t e = grid2view(external.x, external.y, view);
    float gate_ang = points_angle_rad(e.x, e.y, i.x, i.y);
    viz_text((i.x + e.x)/2, (i.y + e.y)/2, 15, RGBA(1,1,1,1), "%f", gate_ang);

    int point_idx;
    if(fabs(gate_ang) < 0.5f)
        return curve;
    if(gate_ang > 0) {
        point_idx = point1;
        sign = -1;
    } else if(gate_ang < 0) {
        point_idx = point2;
        sign = +1;
        internal = grid.points[point2];
        external = grid.points[point1];
    } 

    viz_line(   grid2view(internal.x, internal.y, view), 
                grid2view(external.x, external.y, view), VIEW_COLOR, 1);

    //calc curve intern
    float s_ang = 0;
    for(int i=0; i<6; i++) {
        int id = point_idx + i*sign;
        if(id <0 || id > grid.points_n-1)
            break;
        point_t s0 = grid.points[id];
        float_point_t a  = grid2view(internal.x, internal.y, view);
        float_point_t b = grid2view(s0.x, s0.y, view);
        float ang = points_angle_rad(a.x, a.y, b.x, b.y) - M_PI/2*sign;
        if(i == 0)
            s_ang = ang;
        else
            s_ang =  (s_ang + ang)/2;
    } 

    //reach opposite wall
    float_point_t int_v = grid2view(internal.x, internal.y, view);
    float_point_t opp_v, l_v;
    float width = 0;
    for(int i=1*nav.inflation +2; i<grid.size; i++) {
        opp_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   opp_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t opp = view2grid(opp_v.x, opp_v.y, view);
        width = i;
        if(getgrid(grid, opp.x, opp.y) > GATE)
            break;
    }
    viz_line(int_v, opp_v, PATH_COLOR, 1);
    
    float_point_t curve_v;
    curve_v.x = int_v.x + cos(s_ang)*view.cell_l*(width*track.sects[track.cur_sect].enter);   
    curve_v.y = int_v.y + sin(s_ang)*view.cell_l*(width*track.sects[track.cur_sect].enter);   
    curve = view2grid(curve_v.x, curve_v.y, view);

    //reach end wall
    s_ang -= M_PI/2*sign;
    for(int i=1*nav.inflation +2; i<grid.size; i++) {
        l_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   l_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t l = view2grid(l_v.x, l_v.y, view);
        if(getgrid(grid, l.x, l.y) > GATE)
            break;
    }
    viz_line(int_v, l_v, PATH_COLOR, 1);

    //sign on the center of curve
    float_point_t center;
    center.x = (opp_v.x + l_v.x)/2;
    center.y = (opp_v.y + l_v.y)/2;
    draw_signal(center, 15, track.sects[track.cur_sect].dir);

    //get curve passed
    int_v.y += 80;
    opp_v.y += 80;
    viz_line(int_v, opp_v, VIEW_COLOR, 1);
    static int time = 0;
    static bool start = false;
    if(start)
        time++;
    if(!start && opp_v.y > view.y + view. l && int_v.y > view.y + view. l)
        start = true;
    if(start && time >= 40 && opp_v.y < view.y + view. l && opp_v.y < view.y + view. l) {
        track.cur_sect = (track.cur_sect +1) % track.sects_n;
        time = 0;
        start = false;
    }

    return curve;
}

void draw_yaw(view_t &view) {
    float size = 50;
    float_point_t center, pointer;
    center.x = view.x + view.l + size/2 +10;
    center.y = view.y + size/2 + 10; 
    pointer.x = center.x + cos(yaw)*size/2;
    pointer.y = center.y + sin(yaw)*size/2;
    viz_line(center, pointer, PATH_COLOR, 1);

    static float prec_yaw = yaw;
    static bool  dir = false;
    float delta = prec_yaw -yaw;
    if(delta > yaw+M_PI/10 && dir == false) {
        dir = true;
        prec_yaw = yaw;
    } else if(delta < yaw-M_PI/10 && dir == true) {
        dir = false;
        prec_yaw = yaw;       
    }
    viz_text(center.x - size/2, center.y + size/2 + 5, 12, VIEW_COLOR, "yaw %f", delta);
    viz_arc(center.x, center.y, size/2, yaw, delta, VIEW_COLOR, 1);
    if(fabs(delta) > M_PI/2 - M_PI/10) {
        prec_yaw = yaw;
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

    car_t car;
    init_car(car, view, nav.zoom);

    int xp = grid.size/2, yp = grid.size - (car.length/10*8)/view.cell_l;
    inflate(grid, xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    int gate_idx = choosegate(grid, xp, yp);
    /*
    grid_line(grid, grid.gates[gate_idx].s.x, grid.gates[gate_idx].s.y,
                    grid.gates[gate_idx].e.x, grid.gates[gate_idx].e.y, EMPTY);
    */
    draw_grid(grid, view);   
    point_t curve = calc_curve(grid, gate_idx, view);
    
    to_x = (grid.gates[gate_idx].s.x + grid.gates[gate_idx].e.x)/2;
    to_y = (grid.gates[gate_idx].s.y + grid.gates[gate_idx].e.y)/2;

    draw_yaw(view); 
    draw_track(track, view);

    //get x and y for start and goal from cells position
    point_t part, goal;
    part.x = xp;    part.y = yp;
    goal.x = to_x;  goal.y = to_y;
    setgrid(grid, goal.x, goal.y, 0);

    path_t path = pathfinding(grid, view, part, goal, curve);

    for(int i=1; i< path.size; i++) {
        viz_circle(path.data[i], 2, PATH_COLOR, 1.0f);
        //viz_line(path.data[i-1], path.data[i], PATH_COLOR, 1);
    }

    float_point_t steer_p = grid2view(part.x, part.y, view); 
    viz_line(steer_p, path.data[path.start], VIEW_COLOR, 2);

    float throttle = 20;
    float steer = points_angle(steer_p.x, steer_p.y, path.data[path.start].x, path.data[path.start].y);
    if(steer > 100) steer = 100;
    if(steer < -100) steer = -100;

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
        dino_nav::FloatPoint point;
        point.x = path.data[i].x;
        point.y = path.data[i].y;

        stat.path.push_back(point);
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

    tf::Quaternion q(   pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}


