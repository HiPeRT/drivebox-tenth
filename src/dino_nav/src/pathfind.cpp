//
// Created by cecco on 24/11/16.
//

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <queue>  
#include <algorithm> 
#include <vector>   


#include "pathfind.h"
#include "dinonav.h"

extern dinonav_t nav;


class node_comp {
    bool reverse;

public:
    node_comp(const bool& revparam=false) {
        reverse=revparam;
    }
  
    bool operator() (const node_t *a, const node_t *b) const {
        if (reverse) 
            return (a->cost<b->cost);
        else 
            return (a->cost>b->cost);
    }
};

const int NEIGHTBOURS_N = 5;
const int NEIGHTBOURS[NEIGHTBOURS_N] = { -100, -50, 0, 50, 100 };
float cell_l = 400.0/20.0;
float car_l = cell_l*4;

void get_neighbours(node_t *node, node_t *nbrs) {

    for(int i=0; i< NEIGHTBOURS_N; i++) {
        node_t *n = &nbrs[i];

        n->steer = NEIGHTBOURS[i];
        n->parent = node;

        n->cost = 0;
        float steer_ang = ((float) n->steer) /100.0 * M_PI/4; 
        n->pos.x = node->pos.x + cos(node->angle + steer_ang)*cell_l;
        n->pos.y = node->pos.y + sin(node->angle + steer_ang)*cell_l;
        n->angle = node->angle + (cell_l/car_l) * tan(steer_ang);
    }
}


path_t pathfinding(grid_t &grid, view_t &view, float_point_t &s, float_point_t &e) {

    std::priority_queue<node_t*, std::vector<node_t*>, node_comp> open;

    const int MAX_IT = 10000;
    static node_t *nodes = NULL;
    if(nodes == NULL)
        nodes = new node_t[MAX_IT*NEIGHTBOURS_N];

    int n_nodes = 0;
    node_t *start = &nodes[n_nodes++];
    start->pos = s;
    start->angle = -M_PI/2;
    start->cost = 0;
    start->parent = NULL;
    start->steer = 0;
    open.push(start);
    
    point_t end_p = view2grid(e.x, e.y, view);


    node_t *n;   
    for(int j=0; open.size()>0 && j<MAX_IT; j++) {
        n = open.top();
        open.pop();

        point_t p = view2grid(n->pos.x, n->pos.y, view);
        if(p.x == end_p.x && p.y == end_p.y)
            break;
        get_neighbours(n, &nodes[n_nodes]);

        for(int i=0; i< NEIGHTBOURS_N; i++) {
            node_t *nbr = &nodes[n_nodes++];

            point_t n = view2grid(nbr->pos.x, nbr->pos.y, view);
            int val = getgrid(grid, n.x, n.y);
            if(val >= 0  && val < WALL) {
                nbr->cost = float(abs(nbr->steer))/100*nav.steer_cost + points_dst(nbr->pos, e)/view.l*nav.dist_gain + val/100*nav.inflation_cost;
                open.push(nbr);
            }
        }
        
    }
    
    path_t path;
    path.size = 0;

    while(n != NULL) {
        path.data[path.size++] = *n;
        n = n->parent;
    }

    return path;
}