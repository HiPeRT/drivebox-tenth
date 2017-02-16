#include <iostream>
#include <math.h>

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

#include "perception.h"

extern dinonav_t nav;
extern track_t track;
extern float estimated_speed;
extern float estimated_acc;

point_t perception(grid_t &grid, car_t &car, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg) {

    discretize_laserscan(grid, view, msg);

    point_t car_pos;
    car_pos.x = grid.size/2;
    car_pos.y = grid.size - (car.length/10*8)/view.cell_l;
    inflate(grid, car_pos.x, car_pos.y, 0, 3);
    
    return car_pos;
}

void discretize_laserscan(grid_t &grid, view_t &view, const sensor_msgs::LaserScan::ConstPtr& msg) {

    float maxd = nav.zoom;
    int quad_l = maxd*2;
    int size = msg->ranges.size();
    double angle = msg->angle_max + M_PI*3/2;

    float noise_toll = 0.10;

    for(int i=0; i<size; i++) {
        float r = msg->ranges[i];
        
        bool evaluate = false;
        float  r_prec, r_succ;
        i>0 ?       r_prec = msg->ranges[i-1] : r_prec = r;
        i<size-1 ?  r_succ = msg->ranges[i+1] : r_prec = r;
        if(fabs(r - r_prec) < noise_toll || fabs(r - r_succ) < noise_toll)
            evaluate = true;

        if(i==size/2)
            grid.middle_id = grid.points_n;

        if(evaluate) {
            //quad_l : view_l = r : view_r
            //coodianates of the sigle ray
            float view_r = r*view.l/quad_l;
            float x = view.l/2 + cos(angle) * view_r;
            float y = view.l + sin(angle) * view_r;

            //coordinates of the corrispondent cell
            int grid_x = x / view.cell_l;
            int grid_y = y / view.cell_l;
            if(setgrid(grid, grid_x, grid_y, WALL)) {

                int n = grid.points_n;
                if( n == 0 || ( n>0                            &&
                                grid.points[n-1].x != grid_x   ||
                                grid.points[n-1].y != grid_y       )) {

                    grid.points[n].x = grid_x;
                    grid.points[n].y = grid_y;
                    grid.points_n++;
                }
            }
            inflate(grid, grid_x, grid_y, INFLATED, nav.inflation);
        }
        //if(i>0 && (last_x != grid_x || last_y != grid_y)) {
        //    grid_line(grid, grid_x, grid_y, last_x, last_y, GATE);
        //}
        angle -= msg->angle_increment;
    }

    for(int i=1; i<grid.points_n; i++) {
        point_t p = grid.points[i];
        point_t prev = grid.points[i-1];
        if(prev.x != p.x || prev.y != p.y) {
            grid_line(grid, p.x, p.y, prev.x, prev.y, GATE);
        }
    }
}