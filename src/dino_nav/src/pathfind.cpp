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

path_t pathfinding(grid_t &grid, view_t &view, point_t &s, point_t &e) {

    PQI open;
    int status[grid.size*grid.size];
    for(int i=0; i<grid.size*grid.size; i++)
        status[i] = 0; 

    static node_t *nodes = NULL;
    if(nodes == NULL)
        nodes = new node_t[MAX_ITER*4];


    point_t curve = view2grid(viz_mouse().x, viz_mouse().y, view);

    int n_nodes = 0;
    node_t *start = &nodes[n_nodes++];
    start->pos = s;
    start->cost = 0;
    start->parent = NULL;
    start->dir = UP;
    open.push(start);
    
    node_t *n;
    node_t *good_end = NULL;   
    int j;
    for(j=0; open.size()>0 && j<MAX_ITER; j++) {
        n = open.top();
        open.pop();
        status[node_id(grid, n)] = -1;

        point_t p = n->pos;
        if(p.x == e.x && p.y == e.y) {
            if(good_end == NULL || good_end->cost > n->cost) {
                good_end = n;
                continue;
            }
        }
        get_neighbours(n, &nodes[n_nodes], n_nodes);

        for(int i=0; i< 3; i++) {
            node_t *nbr = &nodes[n_nodes++];
            viz_line(grid2view(p.x, p.y, view), grid2view(nbr->pos.x, nbr->pos.y, view), RGBA(1,0,0,0.1f), 1);

            int val = getgrid(grid, nbr->pos.x, nbr->pos.y);
            if(val == 0 && status[node_id(grid, nbr)] >= 0) {

                if(nbr->pos.y < curve.y) {
                    nbr->cost = view.cell_l + (abs(nbr->pos.x - e.x) + abs(nbr->pos.y - e.y))*view.cell_l;
                } else {
                    nbr->cost = view.cell_l + (abs(nbr->pos.x - curve.x) + abs(nbr->pos.y - curve.y))*view.cell_l;
                }

                if(nbr->dir == nbr->parent->dir)
                    nbr->cost += view.cell_l/2;
                   
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
    
    n = good_end;
    path_t path;
    path.size = 0;

    while(n != NULL) {
        path.data[path.size++] = grid2view(n->pos.x, n->pos.y, view);
        n = n->parent;
    }
    printf("iter %d, size: %d\n", j, path.size);

    return path;
}