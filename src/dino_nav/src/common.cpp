#include <math.h> 

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