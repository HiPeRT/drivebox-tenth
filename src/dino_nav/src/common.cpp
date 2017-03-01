#include <math.h> 
#include "common.h"

/**
    Compute angle beetwen 2 points in term of car steering (-100; +100)
*/
float points_angle(float x_part, float y_part, float x_goal, float y_goal) {
 
    float ang = atan2(y_goal - y_part, x_goal - x_part)*180/M_PI +90;
    if(ang > 180) ang -= 360;
    // ang : 45 = new_ang : 100
    ang = ang*100/45;
 
    return ang;
}

/**
    Compute angle beetwen 2 points in rad
*/
float points_angle_rad(float x_part, float y_part, float x_goal, float y_goal) {

    float ang = atan2(y_goal - y_part, x_goal - x_part);
    return ang;
}

/**
    Rotate point p around origin of theta angle (rad)
*/
void rotate_point(float_point_t &p, float_point_t &o, float theta) {
    int x = o.x + (p.x - o.x)*cos(theta) - (p.y - o.y)*sin(theta);
    int y = o.y + (p.y - o.y)*cos(theta) + (p.x - o.x)*sin(theta);

    p.x = x; p.y = y;
}

/**
    Get max value of an array of float
*/
float get_max_value(const float *a, int dim) {
    float max = a[0];
    for(int i=1; i<dim; i++)
        if(a[i] > max) max = a[i];
    return max;
}

/**
    Compute points distance 
*/
float point_dst(float_point_t &a, float_point_t &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

/*
    Tell if a point is in front or back of a segment
    r>0 = FRONT, r<0 = BACK, r == ALINE 
*/
float point_is_front(segment_t &s, float_point_t &p){
 
    float r = ((s.b.x - s.a.x)*(p.y - s.a.y) - (s.b.y - s.a.y)*(p.x - s.a.x));
    return r;
}
 
/**
    Clamp a float value beetwen min and max
*/
float fclamp(float val, float min, float max) {
    if(val > max) return max;
    else if(val < min) return min;
    else return val;
}