#include "ros/ros.h"
#include "viz.h"

int main(int argc, char **argv){

	//Inizializzo il nodo dinomap
	ros::init(argc, argv, "dinomap");

	//ros::NodeHandle n;

	viz_init(700,700);
    while(ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();

	return 0;
}