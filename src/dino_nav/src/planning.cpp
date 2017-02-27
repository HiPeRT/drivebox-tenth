#include <iostream>
#include <math.h>

#ifdef NOVIZ
    #include "dummyviz.h"
#else
    #include "viz.h"
#endif

#include "pathfind.h"
#include "planning.h"


void planning(dinonav_t &nav) {
 
    nav.goal_pos.x = -1;
    nav.goal_pos.y = -1;

    int gate_idx = choosegate(nav.grid, nav.car_pos.x, nav.car_pos.y);
    nav.curve = calc_curve(nav.grid, gate_idx, nav.view, nav.car, nav.track, nav.conf);

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


segment_t calc_curve(grid_t &grid, int gate_idx, 
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

/*
    int point_idx;
    if(point1 < grid.middle_id && point2 < grid.middle_id) {
        sign = -1;
        internal = grid.points[point1];
        external = grid.points[point2];
        point_idx = point1;
        viz_circle(grid2view(internal.x, internal.y, view), 2, PATH_COLOR, 1);

    } else if (point1 >= grid.middle_id && point2 >= grid.middle_id) {
        sign = +1;
        internal = grid.points[point2];
        external = grid.points[point1];
        point_idx = point2;
        viz_circle(grid2view(internal.x, internal.y, view), 2, PATH_COLOR, 1);

    } else {
        return;
    }*/

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

    viz_line(   grid2view(internal.x, internal.y, view), 
                grid2view(external.x, external.y, view), VIEW_COLOR, 1);

    //calc curve intern
    float s_ang = 0;
    for(int i=0; i<6; i++) {
        int id = point_idx + i*sign;
        if(id <0 || id > grid.points_n-1)
            break;
        point_t s0 = grid.points[id];
        float_point_t a  = grid2view(internal.x, internal.y, view);
        float_point_t b = grid2view(s0.x, s0.y, view);
        float ang = points_angle_rad(a.x, a.y, b.x, b.y) - M_PI/2*sign;
        if(i == 0)
            s_ang = ang;
        else
            s_ang =  (s_ang + ang)/2;
    } 

    //reach opposite wall
    float_point_t int_v = grid2view(internal.x, internal.y, view);
    float_point_t opp_v, l_v;
    float width = 0;
    for(int i=1*conf.inflation +2; i<grid.size; i++) {
        opp_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   opp_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t opp = view2grid(opp_v.x, opp_v.y, view);
        width = i;
        if(getgrid(grid, opp.x, opp.y) > GATE)
            break;
    }
    viz_line(int_v, opp_v, PATH_COLOR, 1);

    curve.a.x = int_v.x;
    curve.a.y = int_v.y;   
    curve.b.x = int_v.x + cos(s_ang)*(car.width*track.sects[track.cur_sect].enter);   
    curve.b.y = int_v.y + sin(s_ang)*(car.width*track.sects[track.cur_sect].enter);   
    curve.dir = sign;

    //reach end wall
    s_ang -= M_PI/2*sign;
    for(int i=1*conf.inflation +2; i<grid.size; i++) {
        l_v.x = int_v.x + cos(s_ang)*view.cell_l*i;   l_v.y = int_v.y + sin(s_ang)*view.cell_l*i;    
        point_t l = view2grid(l_v.x, l_v.y, view);
        if(getgrid(grid, l.x, l.y) > GATE)
            break;
    }
    viz_line(int_v, l_v, PATH_COLOR, 1);

    /*anticipate brake
    if(estimated_speed > 1) {
        curve.b.x += cos(s_ang + M_PI)*car.width*(estimated_speed-1);   
        curve.b.y += sin(s_ang + M_PI)*car.width*(estimated_speed-1); 
    }*/

    //sign on the center of curve
    float_point_t center;
    center.x = (opp_v.x + l_v.x)/2;
    center.y = (opp_v.y + l_v.y)/2;
    draw_signal(center, 15, track.sects[track.cur_sect].dir);

    return curve;
}