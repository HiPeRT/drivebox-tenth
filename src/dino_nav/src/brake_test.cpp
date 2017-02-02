#include "ros/ros.h"
#include "viz.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "common.h"

struct vquad_t {
    float x, y, l;
};

enum state_e { PREPARE, START, BREAK };

ros::Publisher drive_pub;

float mean_ray(std::vector<float> v, int idx, int l) {
    float sum = 0;
    for(int i=idx-l; i<=idx+l; i++)
        sum += v[i];
    return sum/(idx*2+1);
}

void run_test(float &throttle, float &steer, float wall_dist, vquad_t &view) {

    const float start_dist = 10;
    const float min_dist = 3;

    static state_e state = PREPARE;
    static int test = 0; 

    float speed = 0;

    switch(state) {

    case PREPARE:
        viz_text(view.x + view.l + 20, view.y +120, 15, VIEW_COLOR, "PREPARE");
        if(wall_dist < start_dist) {
            throttle = -10;
            steer = -steer;
        } else {
            state = START;
        }
        break;

    case START:
        viz_text(view.x + view.l + 20, view.y +120, 15, VIEW_COLOR, "PREPARE -> START");
        if(wall_dist > min_dist)
            throttle = 50;
        else
            state = BREAK;
        break;

    case BREAK:
        viz_text(view.x + view.l + 20, view.y +120, 15, VIEW_COLOR, "PREPARE -> START -> BREAK");
        if(speed > 0)
            throttle = -50;
        else
            state = PREPARE;
        break;
    }

}

void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
     
    viz_clear();
    vquad_t view;
    view.x = 10;
    view.y = 10;
    view.l = 400;

    int size = msg->ranges.size();
    float front_ray = mean_ray(msg->ranges, size/2, 2);
    int ray_wideness = 40;

    float left_ray = mean_ray(msg->ranges, size/2 + ray_wideness, 2);
    float right_ray = mean_ray(msg->ranges, size/2 - ray_wideness, 2);

    float view_convert = view.l/10;
    front_ray = front_ray*view_convert;
    left_ray = left_ray*view_convert;
    right_ray = right_ray*view_convert;

    float_point_t wall_a, wall_b;
    wall_a.x = view.x;          wall_a.y = view.y;
    wall_b.x = view.x + view.l; wall_b.y = view.y;
    viz_line(wall_a, wall_b, VIEW_COLOR, 1);
    float_point_t wall_middle;
    wall_middle.x = (wall_a.x + wall_b.x)/2;
    wall_middle.y = (wall_a.y + wall_b.y)/2;
    viz_circle(wall_middle, 3, PATH_COLOR, 1);

    float alpha = msg->angle_increment*ray_wideness;
    float a = right_ray;
    float b = left_ray;    
    float d = front_ray;
    float beta = asin( (b*sin(alpha)) / sqrt(b*b + d*d - 2*b*d*cos(alpha)));
    if(b < a)
        beta = M_PI/2 - (beta - M_PI/2);


    float_point_t car_p;
    car_p.x = wall_middle.x + cos(beta)*front_ray;
    car_p.y = wall_middle.y + sin(beta)*front_ray;
    viz_circle(car_p, 3, CAR_COLOR, 1);
    viz_line(car_p, wall_middle, CAR_COLOR, 1);

    float_point_t left_hit, right_hit;
    float dira = points_angle_rad(car_p.x, car_p.y, wall_middle.x, wall_middle.y);
    left_hit.x = car_p.x + cos(dira - alpha)*b;
    left_hit.y = car_p.y + sin(dira - alpha)*b;
    right_hit.x = car_p.x + cos(dira + alpha)*a;
    right_hit.y = car_p.y + sin(dira + alpha)*a;
    viz_line(car_p, left_hit, CAR_COLOR, 1);
    viz_line(car_p, right_hit, CAR_COLOR, 1);

    float angle = dira + M_PI/2;
    float wall_dist = (car_p.y - view.y)/view_convert;
    viz_text(view.x + view.l + 20, view.y +20, 20, VIEW_COLOR, "wall angle: %f", angle);
    viz_text(view.x + view.l + 20, view.y +40, 20, VIEW_COLOR, "wall dist: %f mt.", wall_dist);

    float throttle = 0;
    float steer = points_angle(car_p.x, car_p.y, wall_middle.x, wall_middle.y);
    steer != steer ? steer = 0 : steer = -fclamp(steer, -100, 100);

    run_test(throttle, steer, wall_dist, view);

    viz_text(view.x + view.l + 20, view.y +60, 20, VIEW_COLOR, "steer: %f", steer);
    viz_text(view.x + view.l + 20, view.y +80, 20, VIEW_COLOR, "throttle: %f", throttle);

    race::drive_param m;
    m.velocity = throttle;
    m.angle = steer;
    drive_pub.publish(m);

    viz_flip();
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "dinonav");

    ros::NodeHandle n;

    ros::Subscriber ssub = n.subscribe("scan", 1, laser_recv);
    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);

    viz_init();
    while(ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();
    return 0;
}
