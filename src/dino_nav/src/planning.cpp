#include <iostream>
#include <math.h>

#include "planning.h"
#include "pathfind.h"

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

void planning(dinonav_t &nav) {
 
    nav.goal_pos.x = -1;
    nav.goal_pos.y = -1;

    int gate_idx = choosegate(nav.grid, nav.car_pos.x, nav.car_pos.y);
    nav.curve = calc_curve(nav.grid, gate_idx, grid2view(nav.car_pos.x, nav.car_pos.y, nav.view), 
                            nav.view, nav.car, nav.track, nav.conf);

    nav.goal_pos.x = (nav.grid.gates[gate_idx].s.x + nav.grid.gates[gate_idx].e.x)/2;
    nav.goal_pos.y = (nav.grid.gates[gate_idx].s.y + nav.grid.gates[gate_idx].e.y)/2;

    setgrid(nav.grid, nav.goal_pos.x, nav.goal_pos.y, 0);

    nav.path = pathfinding(nav.grid, nav.view, nav.car, nav.car_pos, nav.goal_pos, nav.curve);
}


void draw_signal(float_point_t center, float r, dir_e d) {

    viz_circle(center, r, RGBA(1,1,1,1), 0);
    viz_circle(center, r, RGBA(1,0,0,1), r/5);
    float_point_t i, e;
    i.x = center.x - r/1.5;
    e.x = center.x + r/1.5;
    i.y = e.y = center.y; 
    viz_line(i, e, RGBA(0,0,0,1), r/10);
    
    float_point_t a, b;
    if(d == LEFT) {
        a.x = i.x + r/3;
        b.x = i.x + r/3;
    } else {
        i.x = e.x;
        a.x = i.x - r/3;
        b.x = i.x - r/3;
    }
    
    a.y = i.y - r/3;
    b.y = i.y + r/3;
    viz_triangle(i, a, b, RGBA(0,0,0,1), 0);
}


int choosegate(grid_t &grid, int px, int py) {

    int max_dim = 0;
    int idx = 0;
    for(int i=0; i<grid.gates_n; i++) {
        if(grid.gates[i].dim > max_dim)  {
            max_dim = grid.gates[i].dim;
            idx = i; 
        }
    }
    
    return idx;
}

segment_t calc_curve(grid_t &grid, int gate_idx, float_point_t start,
    view_t &view, car_t &car, track_t &track, conf_t &conf) {
    
    segment_t curve;
    curve.a.x = -1; curve.a.y = -1;
    curve.b.x = -1; curve.b.y = -1;

    point_t g1 = grid.gates[gate_idx].s;
    point_t g2 = grid.gates[gate_idx].e;

    point_t internal, external;
    int sign;

    int point1=-1, point2=-1;
    for(int i=0; i<grid.points_n; i++) {
        if( (grid.points[i].x == g1.x && grid.points[i].y == g1.y)     ||
            (grid.points[i].x == g2.x && grid.points[i].y == g2.y)    ) {
            
            if(point1 <0)
                point1 = i;
            else if(point2 <0) {
                point2 = i;
                break;
            }
        }
    }

    internal = grid.points[point1];
    external = grid.points[point2];
    float_point_t i = grid2view(internal.x, internal.y, view);
    float_point_t e = grid2view(external.x, external.y, view);
    float gate_ang = points_angle_rad(e.x, e.y, i.x, i.y);
    viz_text((i.x + e.x)/2, (i.y + e.y)/2, 15, RGBA(1,1,1,1), "%f", gate_ang);

    int point_idx;
    if(fabs(gate_ang) < 0.5f)
        return curve;
    if(gate_ang > 0) {
        point_idx = point1;
        sign = -1;
    } else if(gate_ang < 0) {
        point_idx = point2;
        sign = +1;
        internal = grid.points[point2];
        external = grid.points[point1];
    } 


    curve.a = grid2view(internal.x, internal.y, view);

    viz_line(   curve.a, 
                grid2view(external.x, external.y, view), VIEW_COLOR, 1);

    float r = car.width*track.sects[track.cur_sect].enter;
    viz_circle(curve.a, r, PATH_COLOR, 1);

    float_point_t tg1, tg2, tg; 
    if(find_circle_tang(curve.a, r, start, tg1, tg2)) {
        if(gate_ang < 0)
            tg = tg1;
        else
            tg = tg2;

        viz_circle(tg, 10, PATH_COLOR, 1);
        curve.b = tg;
    } else {
        curve.b.x = -1; curve.b.y = -1;
        return curve;
    }

    viz_circle(grid2view(internal.x, internal.y, view), 10, PATH_COLOR, 0);

    return curve;
}