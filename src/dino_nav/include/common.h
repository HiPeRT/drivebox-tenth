#ifndef COMMON_H
#define COMMON_H

struct point_t {
    int x, y;
};

struct float_point_t {
    float x, y;
};

float points_angle(float x_part, float y_part, float x_goal, float y_goal);
float points_angle_rad(float x_part, float y_part, float x_goal, float y_goal);

 void rotate_point(float_point_t &p, float_point_t &o, float theta);

#endif //COMMON_H