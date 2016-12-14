#include "ros/ros.h"
#include "dinonav.h"

#include "race/drive_param.h"
#include "std_msgs/Float32.h"
#include "dino_nav/Stat.h"

extern ros::Publisher drive_pub, stat_pub; //speed_pub;

int main(int argc, char **argv) {

    ros::init(argc, argv, "dinonav");

    ros::NodeHandle n;

    ros::Subscriber ssub = n.subscribe("scan", 1, laser_recv);
    ros::Subscriber psub = n.subscribe("pose_stamped", 1, pose_recv);
    ros::Subscriber osub = n.subscribe("zed/odom", 1, odom_recv);

    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);
    stat_pub = n.advertise<dino_nav::Stat>("dinonav/stat", 1);
    //speed_pub = n.advertise<std_msgs::Float32>("dinonav/speed", 1);

    dynamic_reconfigure::Server<dino_nav::DinonavConfig> server;
    dynamic_reconfigure::Server<dino_nav::DinonavConfig>::CallbackType f;

    f = boost::bind(&reconf, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}
