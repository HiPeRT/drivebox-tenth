#include <iostream>
#include <math.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "dino_nav/Stat.h"

#include "grid.h"
#include "pathfind.h"
#include "dinonav.h"
#include "common.h"
#include "viewer.h"

ALLEGRO_FONT *font;

void map_recv(const dino_nav::Stat::ConstPtr& msg) {
    ROS_INFO("stat recived");
    al_clear_to_color(al_map_rgb(0,0,0));

    int grid_dim = msg->grid_size;
    int grid[GRID_MAX_DIM*GRID_MAX_DIM];
    
    view_t view;
    view.x = 10; view.y = 10;
    view.l = 512;
    view.cell_l = view.l / float(grid_dim);

    for(int i=0; i<grid_dim; i++) {
        al_draw_line(view.x + i * view.cell_l, view.y, view.x + i * view.cell_l, view.y + view.l, VIEW_GRID_COLOR, 1);
        al_draw_line(view.x, view.y + i * view.cell_l, view.x + view.l, view.y + i * view.cell_l, VIEW_GRID_COLOR, 1);

        for (int j = 0; j < grid_dim; j++)
            grid[i*grid_dim +j] = msg->grid[i*grid_dim + j];
    }

    al_draw_rectangle(view.x, view.y, view.x + view.l, view.y + view.l, VIEW_COLOR, 1);

    for(int i=0; i<grid_dim; i++) {
        for (int j = 0; j < grid_dim; j++) {
            int val = grid[i*grid_dim +j];
            ALLEGRO_COLOR col;

            if (val == WALL) {
                col = WALL_COLOR;
            } else if(val == INFLATED) {
                col = INFLATED_COLOR;  
            } else if(val == GATE) {
                col = GATE_COLOR;  
            } else {
                continue;
            }

            al_draw_filled_rectangle(   view.x + view.cell_l * j, view.y + view.cell_l * i, view.x + view.cell_l * (j + 1),
                                        view.y + view.cell_l * (i + 1), col);
        }
    }

    car_t car;
    init_car(car, view);
    
    int xp = grid_dim/2, yp = grid_dim - car.length/view.cell_l;

    al_draw_rectangle(  view.x + view.cell_l/2 + view.l/2 - car.width/2, view.y + view.l, 
                        view.x + view.cell_l/2 + view.l/2 + car.width/2, view.y + view.l - car.length, 
                        CAR_COLOR,1);
                      
    for(int i=0; i<msg->path_size-1; i++) {
        int x1 = msg->path[i].x, y1 = msg->path[i].y, x2 = msg->path[i+1].x, y2 = msg->path[i+1].y;
        
        al_draw_filled_rectangle(view.x + view.cell_l * x1, view.y + view.cell_l * y1, view.x + view.cell_l * (x1 + 1),
                                 view.y + view.cell_l * (y1 + 1), PATH_GRID_COLOR);
        x1+=1;                         
        al_draw_filled_rectangle(view.x + view.cell_l * x1, view.y + view.cell_l * y1, view.x + view.cell_l * (x1 + 1),
                                 view.y + view.cell_l * (y1 + 1), PATH_GRID_COLOR);
        x1-=2;
        al_draw_filled_rectangle(view.x + view.cell_l * x1, view.y + view.cell_l * y1, view.x + view.cell_l * (x1 + 1),
                            view.y + view.cell_l * (y1 + 1), PATH_GRID_COLOR);
        x1+=1;                        
        float_point_t s = grid2view(x1, y1, view);
        float_point_t e = grid2view(x2, y2, view);
        /*
        float a = points_angle_rad(s.x, s.y, e.x, e.y);
        float l = car.width/2;
        al_draw_line(e.x, e.y, e.x + cos(a+M_PI/2)*l, e.y + sin(a+M_PI/2)*l, scan_col, 1);
        al_draw_line(e.x, e.y, e.x + cos(a-M_PI/2)*l, e.y + sin(a-M_PI/2)*l, scan_col, 1);
        */
        al_draw_line(s.x, s.y, e.x, e.y, PATH_COLOR, 1);
    }                    

    int x = msg->path[msg->path_start].x, y = msg->path[msg->path_start].y;
    float_point_t s = grid2view(xp, yp, view);
    float_point_t e = grid2view(x, y, view);
    al_draw_line(s.x, s.y, e.x, e.y, VIEW_COLOR, 2);


    al_draw_textf(font, VIEW_COLOR, view.x, view.y + view.l, 0, "%f", msg->speed);
    al_flip_display();
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "dinonav_viewer");

    ros::NodeHandle n;

    ros::Subscriber m_sub = n.subscribe("dinonav/stat", 1,   map_recv);

    ALLEGRO_DISPLAY *display = NULL;

    if(!al_init()) {
        fprintf(stderr, "failed to initialize allegro!\n");
        return -1;
    }

    display = al_create_display(600, 600);
    if(!display) {
        fprintf(stderr, "failed to create display!\n");
        return -1;
    }

    al_install_keyboard();

    al_init_primitives_addon();
    al_init_font_addon(); // initialize the font addon
    al_init_ttf_addon();// initialize the ttf (True Type Font) addon

    font = al_load_ttf_font("/usr/share/fonts/truetype/freefont/FreeMono.ttf",20,0 );

    if (!font){
        fprintf(stderr, "Could not load font.\n");
        return -1;
    }

    ALLEGRO_TIMER *timer = al_create_timer(1.0 / 60);
    if(!timer) {
        fprintf(stderr, "failed to create timer!\n");
        return -1;
    }

    ALLEGRO_EVENT_QUEUE *event_queue = al_create_event_queue();
    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_register_event_source(event_queue, al_get_keyboard_event_source());
    al_register_event_source(event_queue, al_get_timer_event_source(timer));

    bool redraw = false;
    al_start_timer(timer);
    while(true) {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);

        if(ev.type == ALLEGRO_EVENT_TIMER) {
            redraw = true;
        } else if(ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            break;
        } 

        if(redraw && al_event_queue_is_empty(event_queue)) {
            redraw = false;
            ros::spinOnce();
        }
    }

    al_destroy_timer(timer);
    al_destroy_display(display);
    al_destroy_event_queue(event_queue);
    return 0;
}
