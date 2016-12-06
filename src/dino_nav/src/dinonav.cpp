#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"

#include <dynamic_reconfigure/server.h>
#include <dino_nav/DinonavConfig.h>

#include "dinonav.h"
#include "pathfind.h"
#include "common.h"
#include "Grid.h"

ros::Publisher drive_pub, map_pub, speed_pub; //path_pub;

float speed = 0;
int inflation = 0;
int stop_cost = 15;
float zoom = 3;
bool enable = true;

float estimated_speed;


/*
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
*/

void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request");
  speed = config.speed;
  inflation = config.inflation;
  stop_cost = config.stop_cost;
  //grid_dim = config.grid_dim;
  zoom = config.zoom;
  enable = config.enable;
}


/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //ROS_INFO("Scan recived: [%f]", msg->scan_time);

    int size = msg->ranges.size();
    float maxd = zoom;
    int quad_l = maxd*2;

    float view_l = 512;
    float view_x = 10, view_y = 10;

    Grid grid;
    int grid_dim = grid.grid_dim;

    float cell_l = view_l / float(grid_dim);


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
        grid.set(grid_x, grid_y, 1);
        grid.inflate(grid_x, grid_y, 2, inflation);

        if(i>0 && (last_x != grid_x || last_y != grid_y)) {
            grid.line(grid_x, grid_y, last_x, last_y);
        }
        last_x = grid_x;
        last_y = grid_y;
        

        angle -= msg->angle_increment;
    }

    int car_length = (view_l/100)*4 / cell_l;
    int xp = grid_dim/2, yp = grid_dim - car_length;
    grid.inflate(xp, yp, 0, 3);
    int to_x = -1, to_y = -1;
    grid.choosegate(xp, yp, to_x, to_y);
    
    point_t calc_path[MAX_ITER];
    int path_idx = pathfinding(grid, xp, yp, to_x, to_y, calc_path);
    //grid[to_y*grid_dim + to_x] = 100;

    //get x and y for start and goal from cells position
    float x_part = view_x + xp*cell_l + cell_l/2,   y_part = view_y + yp*cell_l + cell_l/2;
    float x_goal = view_x + to_x*cell_l + cell_l/2, y_goal = view_y + to_y*cell_l + cell_l/2;    

    //compute angle for the steer
    float ang  = points_angle(x_part, y_part, x_goal, y_goal);
    
    //compute angle ahead for the speed modulation
    int ah_idx = path_idx -4;
    if(ah_idx < 0) ah_idx = 0;
    point_t ahead_p = calc_path[ah_idx];
    x_goal = view_x + ahead_p.x*cell_l + cell_l/2;
    y_goal = view_y + ahead_p.y*cell_l + cell_l/2;    
    //grid[ahead_p.y*grid_dim + ahead_p.x] = 101;
    float ang2 = points_angle(x_part, y_part, x_goal, y_goal);

    if(enable) {
        race::drive_param m;
        if(fabs(ang2 - ang) > 30 && estimated_speed > 1.5) {
            m.velocity = -10;
            printf("BREAK!!!\n");
        } else {
	    if(fabs(ang2 - ang) < 10 && estimated_speed < 3)
        	m.velocity = speed*2;    
	    else
		m.velocity = speed;
        }

        m.angle = ang;
        if(m.angle >  100) m.angle=100;
        if(m.angle < -100) m.angle=-100;

        drive_pub.publish(m);
    }

    //print_map(grid, grid_dim);
    nav_msgs::OccupancyGrid grid_p;
    grid_p.info.resolution = 0.1;           // float32
    grid_p.info.width      = grid.grid_dim; // uint32
    grid_p.info.height     = grid.grid_dim; // uint32
    std::vector<signed char> vgrd(grid.getGrid(), grid.getGrid()+(grid.grid_dim*grid.grid_dim));
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
    ros::Subscriber ssub = n.subscribe("scan", 1, laser_recv);
    ros::Subscriber psub = n.subscribe("pose_stamped", 1, pose_recv);

    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("dinonav/map", 1);
    speed_pub = n.advertise<std_msgs::Float32>("dinonav/speed", 1);

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
