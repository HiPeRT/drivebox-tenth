#include <iostream>

#include "ros/ros.h"
#include "Dinonav.h"

extern ros::Publisher drive_pub, map_pub, speed_pub; 

int main(int argc, char **argv) {

    ros::init(argc, argv, "dinonav");
    ros::NodeHandle n;
    
    ros::Subscriber ssub = n.subscribe("scan", 1, laser_recv);
    ros::Subscriber psub = n.subscribe("pose_stamped", 1, pose_recv);

    drive_pub   = n.advertise<race::drive_param>("drive_parameters", 1);
    map_pub     = n.advertise<nav_msgs::OccupancyGrid>("dinonav/map", 1);
    speed_pub   = n.advertise<std_msgs::Float32>("dinonav/speed", 1);

    dynamic_reconfigure::Server<dino_nav::DinonavConfig> server;
    dynamic_reconfigure::Server<dino_nav::DinonavConfig>::CallbackType f;

    f = boost::bind(&reconf, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
