#include <math.h> 
#include "common.h"

float points_angle(float x_part, float y_part, float x_goal, float y_goal) {
 
    float ang = atan2(y_goal - y_part, x_goal - x_part)*180/M_PI +90;
    if(ang > 180) ang -= 360;
    // ang : 45 = new_ang : 100
    ang = ang*100/45;
 
    return ang;
 }


 float points_angle_rad(float x_part, float y_part, float x_goal, float y_goal) {
 
    float ang = atan2(y_goal - y_part, x_goal - x_part);
    return ang;
 }

 void rotate_point(float_point_t &p, float_point_t &o, float theta) {
     int x = o.x + (p.x - o.x)*cos(theta) - (p.y - o.y)*sin(theta);
     int y = o.y + (p.y - o.y)*cos(theta) + (p.x - o.x)*sin(theta);
 
     p.x = x; p.y = y;
 }


 float points_dst(float_point_t &a, float_point_t &b) {
     float dx = a.x - b.x;
     float dy = a.y - b.y;
     return sqrt(dx*dx + dy*dy);
 }