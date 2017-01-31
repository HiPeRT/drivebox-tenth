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
#include "viz.h"

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

void get_neighbours(node_t *node, node_t *nbrs, int num) {

    for(int i=0; i< 3; i++) {
        node_t *n = &nbrs[i];

        n->parent = node;
        n->cost = 0;
        
        n->pos.x = node->pos.x;
        n->pos.y = node->pos.y;

        if(i==0) {
            n->dir = LEFT;
            n->pos.x = node->pos.x-1;

        } else if(i==1) {
            n->dir = RIGHT;
            n->pos.x = node->pos.x+1;
        
        } else if(i==2) {
            n->dir = UP;
            n->pos.y = node->pos.y-1;
        }
    }
}

int node_id(grid_t &grid, node_t *n) {
    return n->pos.y*grid.size + n->pos.x;
}

class PQI : public std::priority_queue<node_t*, std::vector<node_t*>, node_comp> {
    public:
        std::vector<node_t*>& impl() { return c; }
};

path_t pathfinding(grid_t &grid, view_t &view, point_t &s, point_t &e, segment_t &curve) {

    PQI open;
    int status[grid.size*grid.size];
    for(int i=0; i<grid.size*grid.size; i++)
        status[i] = 0; 

    point_t crv = view2grid(curve.b.x, curve.b.y, view);
    if(crv.x > grid.size || crv.y > grid.size) {
        crv.x = -1;
        crv.y = -1;
    }

    float_point_t st = grid2view(s.x, s.y, view);
    float dir = point_is_front(curve, st);
    dir /= fabs(dir);

    static node_t *nodes = NULL;
    if(nodes == NULL)
        nodes = new node_t[MAX_ITER*4];

    int n_nodes = 0;
    node_t *start = &nodes[n_nodes++];
    start->pos = s;
    start->cost = 0;
    start->parent = NULL;
    start->dir = UP;
    open.push(start);
    
    node_t *n;
    int j;
    for(j=0; open.size()>0 && j<MAX_ITER; j++) {
        n = open.top();
        open.pop();
        status[node_id(grid, n)] = -1;

        point_t p = n->pos;
        if(e.x -1 <= p.x && e.x +1 >= p.x && e.y -1 <= p.y && e.y +1 >= p.y) {
            break;
        }
        get_neighbours(n, &nodes[n_nodes], n_nodes);

        for(int i=0; i< 3; i++) {
            node_t *nbr = &nodes[n_nodes++];
            float_point_t nbr_v = grid2view(nbr->pos.x, nbr->pos.y, view);
            viz_line(grid2view(p.x, p.y, view), nbr_v, RGBA(1,0,0,0.06f), 1);

            int val = getgrid(grid, nbr->pos.x, nbr->pos.y);
            if(val == 0 && status[node_id(grid, nbr)] >= 0) {

                if(crv.x >0 && crv.y >0) {
                    if(point_is_front(curve, nbr_v)*dir > 0) {
                        float dx1 = nbr->pos.x - crv.x;
                        float dy1 = nbr->pos.y - crv.y;
                        float dx2 = s.x - crv.x;
                        float dy2 = s.y - crv.y;
                        float cross = fabs(dx1*dy2 - dx2*dy1);
                        nbr->cost = cross*0.001;
                    } else {
                        float dx1 = nbr->pos.x - e.x;
                        float dy1 = nbr->pos.y - e.y;
                        float dx2 = crv.x - e.x;
                        float dy2 = crv.y - e.y;
                        float cross = fabs(dx1*dy2 - dx2*dy1);
                        nbr->cost = cross*0.001;
                    }
                } else {
                    float dx1 = nbr->pos.x - e.x;
                    float dy1 = nbr->pos.y - e.y;
                    float dx2 = s.x - e.x;
                    float dy2 = s.y - e.y;
                    float cross = fabs(dx1*dy2 - dx2*dy1);
                    nbr->cost = cross*0.001;
                }
                   
                int stat = status[node_id(grid, nbr)];
                if(stat == 0) {
                    open.push(nbr);
                    status[node_id(grid, nbr)] = 1;
                } else if(stat == 1) {
                    for(int j=0; j<open.size(); j++) {
                        if(node_id(grid, open.impl()[j]) == node_id(grid, nbr) && open.impl()[j]->cost > nbr->cost) {
                            open.impl().erase(open.impl().begin() + j);
                            open.push(nbr);
                        }
                    }
                }
            }
        }
        
    }
    
    path_t path;
    path.size = 0;

    while(n != NULL) {
        path.data[path.size++] = grid2view(n->pos.x, n->pos.y, view);
        n = n->parent;
    }
    path.start = path.size - 18;
    if(path.start < 0)
        path.start = 0;

    //shortcuts path
    for(int i=path.start; i<path.size-2; i++) {
        point_t p = view2grid(path.data[i].x, path.data[i].y, view);

        if(grid_line_control(grid, s.x, s.y, p.x, p.y)) {
            path.start = i;
            break;
        }
    }
    
    return path;
}