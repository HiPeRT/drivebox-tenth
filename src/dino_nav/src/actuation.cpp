#include <iostream>
#include <math.h>

#include "actuation.h"

#include "dinonav.h"
#include "pathfind.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

void actuation(dinonav_t &nav, race::drive_param &drive_msg) {

    point_t part = nav.car_pos;
    point_t goal = nav.goal_pos;

    float_point_t start = grid2view(part.x, part.y, nav.view);
    float_point_t enter = nav.curve.b;
    float_point_t exit = grid2view(goal.x, goal.y, nav.view);
    nav.curve_dst = -1;
    if(nav.curve.b.x >0) {
        viz_circle(enter, 4, CAR_COLOR, 1);
        viz_circle(exit, 4, CAR_COLOR, 1);
        viz_line(start, enter, CAR_COLOR, 1);
        viz_line(enter, exit, CAR_COLOR, 1);
        
        nav.curve_dst = point_dst(start, enter);
        nav.curve_dst = nav.curve_dst*((nav.conf.zoom*2)/nav.view.l);
        viz_text((start.x + enter.x)/2, (start.y + enter.y)/2, 10, RGBA(1,0,1,1), "  %f", nav.curve_dst);
    } else {
        viz_circle(exit, 4, CAR_COLOR, 1);
    }

    nav.steer = calc_steer(start, nav.view, nav.car, nav.grid, nav.path, nav.steer_l);

    nav.throttle = calc_throttle(nav.conf, nav.view, nav.car, nav.track, nav.curve, 
        nav.curve_dst, nav.estimated_speed, nav.estimated_acc, nav.target_acc);
    
    if(nav.throttle > nav.conf.throttle)
        nav.throttle = nav.conf.throttle;

    drive_msg.velocity = nav.throttle;
    drive_msg.angle = nav.steer;
    
}


float calc_steer(float_point_t &start, view_t &view, car_t &car, grid_t &grid, path_t &path, int &steer_l) {
    float steer = 0;
    steer_l = 0;

    for(int i=0; i<path.size; i++) {
        point_t p = view2grid(path.data[i].x, path.data[i].y, view);
        setgrid(grid, p.x, p.y, PATH);
    }

    int best_steer = 0;
    int best_dist = 0;
    for(int j=-100; j<100; j+=2) {
        float_point_t p = start;
        float ang = -M_PI/2;

        float_point_t p0 = p;
        for(int i=0; i<20; i++) {
            float steer_ang = ((float) j) /100.0 * M_PI/4; 

            p.x = p.x + cos(ang + steer_ang)*view.cell_l;
            p.y = p.y + sin(ang + steer_ang)*view.cell_l;
            ang = ang + (view.cell_l/car.length) * tan(steer_ang);
            
            viz_line(p, p0, LPATH_COLOR, 1);
            point_t gp = view2grid(p.x, p.y, view);
            int val = getgrid(grid, gp.x, gp.y);
            if(val == PATH) {
                viz_circle(p, 5, LPATH_COLOR, 1);
                if(i > best_dist) {
                    best_steer = j;
                    best_dist = i;
                }
            } else if(val != EMPTY) {
                break;
            }
            p0 = p;
        }
    }

    float_point_t p = start;
    float ang = -M_PI/2;

    float_point_t p0 = p;
    for(int i=0; i<best_dist; i++) {
        float steer_ang = ((float) best_steer) /100.0 * M_PI/4; 

        p.x = p.x + cos(ang + steer_ang)*view.cell_l;
        p.y = p.y + sin(ang + steer_ang)*view.cell_l;
        ang = ang + (view.cell_l/car.length) * tan(steer_ang);
        viz_line(p, p0, VIEW_COLOR, 2);
        p0 = p;
    }

    steer = best_steer;
    steer_l = best_dist;

    return steer;
}


float calc_throttle(conf_t &conf, view_t &view, car_t &car, track_t &track, segment_t &curve, 
    float curve_dst, float estimated_speed, float estimated_acc, float &target_acc) {

    static float throttle = 0;
    float curve_speed = conf.curve_speed;
        
    curve_dst -= 1;
    float min_dist = (estimated_speed*estimated_speed - curve_speed*curve_speed) / (2 * conf.car_decel);
    if(curve_dst >0 && curve_dst < min_dist && estimated_speed > curve_speed) {
        if(throttle > 0)
            throttle = 0;
        throttle -= 1;
    } else {
        if(throttle < 0)
            throttle = 0;
        throttle++;
    } 

    if(throttle != throttle)
        throttle = 0;
    throttle = fclamp(throttle, -100, 100);

    static int in_curve = 0;
    float_point_t pos;
    pos.x = view.x + view.l/2;
    pos.y = view.y + view.l - conf.ahead_offset - car.length*1;
    
    if(point_is_front(curve, pos)*curve.dir > 0) {
        viz_line(curve.a, curve.b, RGBA(1,0,0,1), 3);
    } else if(curve_dst +1 > 0 && curve_dst +1 < 1) {
        viz_line(curve.a, curve.b, RGBA(0,1,0,1), 3);
        if(in_curve < 0) {
            printf("curve passed\n");
            track.cur_sect = (track.cur_sect +1) % track.sects_n;
            in_curve = 15;
        }
    }
    in_curve--;

    return throttle;
}