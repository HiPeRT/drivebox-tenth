#include "ros/ros.h"
#include "viz.h"
#include "race/drive_param.h"
#include "sensor_msgs/LaserScan.h"
#include "common.h"

struct vquad_t {
    float x, y, l;
};

enum state_e { PREPARE, START, BRAKE, END };


const int MAX_TESTS = 8;
const int MAX_VELS = 1024;
struct test_t {
    float throttle, brake;
    float vels[MAX_VELS], vels_n;
    float speed_reached, start, brake_start, brake_end;
} tests[MAX_TESTS];
int test_num;

ros::Publisher drive_pub;

void print_tests(float_point_t pos, int cur_test) {

    for(int i=0; i<cur_test; i++) {
        test_t *t = &tests[i];
        float offs = 40*i;

        viz_text(pos.x, pos.y + offs, 12, VIEW_COLOR, 
            "TEST %d, throttle: %f  brake: %f", i, t->throttle, t->brake);
        viz_text(pos.x, pos.y + offs +10, 12, VIEW_COLOR, 
            "dists: start %f, brake start: %f  brake end: %f", t->start, t->brake_start, t->brake_end);
         viz_text(pos.x, pos.y + offs +20, 12, VIEW_COLOR, 
            "max speed %f, acc dist %f, brake dist %f", 
            t->speed_reached, t->start - t->brake_start, t->brake_start - t->brake_end);           
    }
}

void print_test_text(int id) {
    test_t *t = &tests[id];

    printf("\n");
    printf("-------- TEST %d --------\n", id);
    printf("throttle: %f  brake: %f\n", t->throttle, t->brake);
    printf("dists: start %f, brake start: %f  brake end: %f\n", 
        t->start, t->brake_start, t->brake_end);
    printf("max speed %f, acc dist %f, brake dist %f\n", 
        t->speed_reached, t->start - t->brake_start, t->brake_start - t->brake_end); 
    printf("-------------------------\n\n");
}

float mean_ray(std::vector<float> v, int idx, int l) {
    float sum = 0;
    
    for(int i=idx-l; i<=idx+l; i++)
        sum += v[i];
    return sum/(l*2+1);
}

void run_test(float &throttle, float &steer, float wall_dist, vquad_t &view) {

    const float start_dist = 6;
    const float min_dist = 3;

    static state_e state = PREPARE;

    static float old_dist = wall_dist;
    static float speed = 0;
    viz_text(view.x + view.l + 20, view.y +100, 20, VIEW_COLOR, "speed: %f", speed);
    
    const int N = 5;
    static int n = 0;
    if(n%N == 0) {
        speed = (old_dist - wall_dist) / (0.025*N);
        old_dist = wall_dist;
    }
    n++;

    static int current_test = 0;
    if(n == 1)
        printf("test %d PREPARE\n", current_test);

    test_t *test = &tests[current_test];

    switch(state) {

    case PREPARE:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: PREPARE");
        if(wall_dist < start_dist) {
            throttle = 0;
            steer = -steer;
        } else {
            state = START;
            tests->start = wall_dist;
            printf("test %d START\n", current_test);
        }
        break;

    case START:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: START");
        if(wall_dist > min_dist) {
            throttle = test->throttle;
        } else {
            state = BRAKE;
            test->brake_start = wall_dist;
            test->speed_reached = speed;
            printf("test %d BRAKE\n", current_test);
        }
        break;

    case BRAKE:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "test %d status: BRAKE");
        if(speed > 0.001 || speed < -0.001) {
            throttle = test->brake;
        } else {
            test->brake_end = wall_dist;
            print_test_text(current_test);
            current_test++;
            if(current_test < test_num) {
                state = PREPARE;
                printf("test %d PREPARE\n", current_test);
            } else {
                state = END;
                printf("ALL TEST ENDED\n");
            }
        }
        break;
    
    case END:
        viz_text(view.x + view.l + 20, view.y +140, 15, VIEW_COLOR, "ALL TESTS EXECUTED");
        break;
    }

    float_point_t test_p;
    test_p.x = view.x + view.l + 20;
    test_p.y = view.y + 180;
    print_tests(test_p, current_test);
}

void laser_recv(const sensor_msgs::LaserScan::ConstPtr& msg) {
     
    viz_clear();
    vquad_t view;
    view.x = 10;
    view.y = 10;
    view.l = 200;

    int size = msg->ranges.size();
    float orig_front_ray = mean_ray(msg->ranges, size/2, 2);
    int ray_wideness = 40;

    float left_ray = mean_ray(msg->ranges, size/2 + ray_wideness, 2);
    float right_ray = mean_ray(msg->ranges, size/2 - ray_wideness, 2);

    float view_convert = view.l/10;
    float front_ray = orig_front_ray*view_convert;
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
    if(beta != beta)
        beta = 0;
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
    float wall_dist = orig_front_ray;
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


void init_tests() {
    test_num = 2; 

    tests[0].throttle =  10;
    tests[0].brake = -5;
    tests[0].vels_n = 0;
    
    tests[1].throttle =  20;
    tests[1].brake = -5;
    tests[1].vels_n = 0;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "dinonav");

    ros::NodeHandle n;

    ros::Subscriber ssub = n.subscribe("scan", 1, laser_recv);
    drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);

    init_tests();

    viz_init();
    while(ros::ok() && viz_update()) {
        ros::spinOnce();
    }

    viz_destroy();
    return 0;
}
