#include "Dinonav.h"

ros::Publisher drive_pub, map_pub, speed_pub; 

float speed;
int   inflation;
int   stop_cost;
float zoom;
bool  enable;
float estimated_speed;

Grid grid;

void reconf(dino_nav::DinonavConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request");
    speed = config.speed;
    inflation = config.inflation;
    stop_cost = config.stop_cost;
    grid.grid_dim = config.grid_dim;
    zoom = config.zoom;
    enable = config.enable;
}


/**
    laserscan callback
*/
void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Scan recived: [%f]", msg->scan_time);

    grid.init();

    int size = msg->ranges.size();
    float maxd = zoom;
    int quad_l = maxd*2;

    float view_l = 512;
    float view_x = 10, view_y = 10;

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

