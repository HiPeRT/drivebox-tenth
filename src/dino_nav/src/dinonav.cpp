#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include <dynamic_reconfigure/server.h>
#include <dino_nav/DinonavConfig.h>

#include "dinonav.h"
#include "pathfind.h"

ros::Publisher drive_pub, map_pub; //path_pub;

float speed = 0;
int inflation = 0;
int stop_cost = 15;
int grid_dim = 100;
bool enable = true;

point_t gates[64][2];
int gates_N;

void print_map(int grid[], int size) {

    for(int i=0; i<size; i++) {
        printf(" ");
        for(int j=0; j<size; j++) {
            int val = grid[i*size +j];
            if(val == 1) printf("X ");
            else if(val == 10) printf("* ");
            else printf("  ");
        }
        printf("\n");
    }
    printf("\n");
}

void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  speed = config.speed;
  inflation = config.inflation;
  stop_cost = config.stop_cost;
  grid_dim = config.grid_dim;
  enable = config.enable;
}

/**
    geterate a line in a matrix from point 1 to 2
*/
void grid_line(int grid[], int x1, int y1, int x2, int y2) {
    //gate search
    int startGx = -1, startGy = -1;
    int endGx = -1,   endGy = -1;

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++)
    {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        if(getgrid(grid, xx, yy) == 0) {
            setgrid(grid, xx, yy, 3);
            if(startGx == -1) {
                startGx = xx; startGy = yy; 
            } else {
                endGx = xx; endGy = yy;
            }
        }
    }

    if(startGx != -1 && endGx != -1) {
        point_t *limit = gates[gates_N];
        limit[0].x = startGx;
        limit[0].y = startGy;
        limit[1].x = endGx;
        limit[1].y = endGy;
        gates_N++;
        //printf("start: %d %d, end: %d %d\n", startGx, startGy, endGx, endGy);
    }
}

/**
    geterate a line in a matrix from point 1 to 2
*/
bool grid_line_control(int grid[], int x1, int y1, int x2, int y2) {

    int dx = x1 - x2;
    int dy = y1 - y2;
    int steps;

    if (abs(dx) > abs(dy))
        steps = abs(dx);
    else
        steps = abs(dy);

    float x_inc = -(float)dx / (float) steps;
    float y_inc = -(float)dy / (float) steps;

    float x = x1, y = y1;
    for(int v=0; v < steps; v++)
    {
        x = x + x_inc;
        y = y + y_inc;
        int xx = x, yy = y;
        int val = getgrid(grid, xx, yy);
        int val1 = getgrid(grid, xx + 1, yy);
        int val2 = getgrid(grid, xx - 1, yy);
        if(val == 1 || val == 2 || val1 == 1 || val1 == 2 || val2 == 1 || val2 == 2)
            return false;

        //setgrid(grid, xx +1, yy, 10);
        //setgrid(grid, xx -1, yy, 10);
        //setgrid(grid, xx, yy, 10);
    }
    return true;
}


/**
    set a grid to value in given position
*/
bool setgrid(int grid[], int x, int y, int value) {
    int pos = y*grid_dim + x;

    if(x<0 || x >= grid_dim || y<0 || y >= grid_dim)
        return false;
    grid[pos] = value;
    return true;
}

int getgrid(int grid[], int x, int y) {
    int pos = y*grid_dim + x;

    if(x<0 || x >= grid_dim || y<0 || y >= grid_dim)
        return -1;
    return grid[pos];
}

/**
    inflate a point with the given spread
          X X X
    X ->  X X X   example with n = 1
          X X X
*/
void inflate(int grid[], int x, int y, int val, int n) {
    if(n == 0)
        return;

    if(getgrid(grid, x, y) != 1)
        setgrid(grid, x, y, val);

    inflate(grid, x-1, y-1, val, n -1);
    inflate(grid, x-1, y, val, n -1);
    inflate(grid, x-1, y+1, val, n -1);
    inflate(grid, x+1, y-1, val, n -1);
    inflate(grid, x+1, y, val, n -1);
    inflate(grid, x+1, y+1, val, n -1);
    inflate(grid, x, y-1, val, n -1);
    inflate(grid, x, y+1, val, n -1);
}


void choosegate(int px, int py, int &to_x, int &to_y) {

    float dst = -1; //min dst
    int gt = -1;

    for(int i=0; i<gates_N; i++) {
        for(int j=0; j<2; j++) {
            
            point_t p = gates[i][j];
            float dx = px - p.x, dy = py - p.y; 
            float d = sqrt(dx*dx + dy*dy); 
            if(d > dst) {
                dst = d;
                gt = i;
            }
        }
    }

    if(gt != -1) {
        point_t s = gates[gt][0];
        point_t e = gates[gt][1];

        to_x = (s.x + e.x)/2;
        to_y = (s.y + e.y)/2;
    } 
}


/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ROS_INFO("Scan recived: [%f]", msg->scan_time);

    int size = msg->ranges.size();
    int maxd = msg->range_max/4;
    int quad_l = maxd*2;

    float view_l = 512;
    float view_x = 10, view_y = 10;

    float cell_l = view_l / float(grid_dim);
    int grid[GRID_MAX_DIM*GRID_MAX_DIM];

    //init grid
    for(int i=0; i<grid_dim*grid_dim; i++)
        grid[i] = 0;
    gates_N = 0;

    int last_x, last_y;
    double angle = msg->angle_max + M_PI*3/2;
    for(int i=0; i<size; i++) {
        float r = msg->ranges[i];

        //quad_l : view_l = r : view_r
        //coodianates of the sigle ray
        float view_r = r*view_l/quad_l;
        float x = view_l/2 + cos(angle) * view_r;
        float y = view_l + sin(angle) * view_r;

        //coordinates of the corrispondent cell
        int grid_x = x / cell_l;
        int grid_y = y / cell_l;
        setgrid(grid, grid_x, grid_y, 1);
        inflate(grid, grid_x, grid_y, 2, inflation);

        if(i>0 && (last_x != grid_x || last_y != grid_y)) {
            grid_line(grid, grid_x, grid_y, last_x, last_y);
        }
        last_x = grid_x;
        last_y = grid_y;
        

        angle -= msg->angle_increment;
    }

    //path grid
    int path[GRID_MAX_DIM*GRID_MAX_DIM];

    int car_length = (view_l/100)*4 / cell_l;
    int xp = grid_dim/2, yp = grid_dim - car_length;
    inflate(grid, xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    choosegate(xp, yp, to_x, to_y);
    pathfinding(path, grid, xp, yp, to_x, to_y);
    grid[to_y*grid_dim + to_x] = 100;

    //get x and y for start and goal from cells position
    float x_part = view_x + xp*cell_l + cell_l/2,   y_part = view_y + yp*cell_l + cell_l/2;
    float x_goal = view_x + to_x*cell_l + cell_l/2, y_goal = view_y + to_y*cell_l + cell_l/2;    

    //compute angle and publish drive message 
    float ang = atan2(y_goal - y_part, x_goal - x_part)*180/M_PI +90;
    if(ang > 180) ang -= 360;
    // ang : 45 = new_ang : 100
    ang = ang*100/45;

    if(enable) {
        race::drive_param m;
        m.velocity = speed;
        m.angle = ang;
        if(m.angle >  100) m.angle=100;
        if(m.angle < -100) m.angle=-100;

        drive_pub.publish(m);
    }

    //print_map(grid, grid_dim);
    nav_msgs::OccupancyGrid grid_p;
    grid_p.info.resolution = 0.1;      // float32
    grid_p.info.width      = grid_dim; // uint32
    grid_p.info.height     = grid_dim; // uint32
    std::vector<signed char> vgrd(grid, grid+(grid_dim*grid_dim));
    grid_p.data = vgrd;
    map_pub.publish(grid_p);

    /*
    nav_msgs::OccupancyGrid path_p;
    path_p.info.resolution = 0.1;      // float32
    path_p.info.width      = grid_dim; // uint32
    path_p.info.height     = grid_dim; // uint32
    std::vector<signed char> vpth(path, path+(grid_dim*grid_dim));
    path_p.data = vpth;
    path_pub.publish(path_p);
    */
}


int main(int argc, char **argv) {
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "listener");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    ros::Subscriber sub = n.subscribe("scan", 1, laser_recv);
    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("dinonav/map", 1);
    //path_pub = n.advertise<nav_msgs::OccupancyGrid>("dinonav/path", 1);
    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */

    dynamic_reconfigure::Server<dino_nav::DinonavConfig> server;
    dynamic_reconfigure::Server<dino_nav::DinonavConfig>::CallbackType f;

    f = boost::bind(&reconf, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
