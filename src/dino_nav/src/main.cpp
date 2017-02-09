#include "ros/ros.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif
#include "dinonav.h"

#include "race/drive_param.h"
#include "std_msgs/Float32.h"
#include "dino_nav/Stat.h"

#include <tinyxml.h>
#include <ros/package.h>

extern ros::Publisher drive_pub, stat_pub; //speed_pub;
extern track_t track;

bool load_track(const char *path) {

    TiXmlDocument doc(path);

    if(doc.LoadFile()) {
        printf("reading track: %s\n", path);
        track.cur_sect = 0;

        int i=0;
        TiXmlElement * sect = doc.FirstChildElement()->FirstChildElement();
        while(sect != NULL) {
            std::cout<<"Sector "<<i<<"\t";
            
            sect->QueryFloatAttribute("l", &track.sects[i].l);
            sect->QueryFloatAttribute("enter", &track.sects[i].enter);
            sect->QueryFloatAttribute("exit", &track.sects[i].exit);
            sect->QueryFloatAttribute("vel", &track.sects[i].vel);

            if(strcmp(sect->Attribute("dir"), "left") == 0) 
                track.sects[i].dir = LEFT;
            else
                track.sects[i].dir = RIGHT;

            std::cout<<"l: "<<track.sects[i].l<<"\t vel: "<<track.sects[i].vel<<"\t";
            std::cout<<" ent: "<<track.sects[i].enter<<"\t exit: "<<track.sects[i].exit;
            std::cout<<"\t dir: "<<track.sects[i].dir;
            std::cout<<"\n";
            sect = sect->NextSiblingElement();
            i++;
            track.sects_n++;
        }
        return true;
    } else {
        printf("unable to load track: %s\n", path);
        return false;
    }

}


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

    std::string path = ros::package::getPath("dino_nav");
    path = path + "/tracks/lica.xml";
    if(!load_track(path.c_str()))
        return -1;

    viz_init(700,700);
    while(ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();
    return 0;
}
