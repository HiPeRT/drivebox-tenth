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

ALLEGRO_FONT *font;

float estimated_speed = 0;

void speed_recv(const std_msgs::Float32::ConstPtr& msg) {

    estimated_speed = msg->data;
}


void map_recv(const dino_nav::Stat::ConstPtr& msg) {
    ROS_INFO("stat recived");
    al_clear_to_color(al_map_rgb(0,0,0));

    int grid_dim = msg->grid_size;
    int grid[GRID_MAX_DIM*GRID_MAX_DIM];
    
    float alpha = 0.3f;
    ALLEGRO_COLOR scan_col = al_map_rgba_f(1.0f, 0, 0, 1.0f);
    ALLEGRO_COLOR obst_col = al_map_rgba_f(1.0f*alpha,1.0f*alpha, 0, alpha);
    ALLEGRO_COLOR quad_col = al_map_rgba_f(0,1.0f,0, 1.0f);
    ALLEGRO_COLOR path_col = al_map_rgba_f(0,1.0f*alpha,1.0f*alpha,alpha);
    ALLEGRO_COLOR gate_col = al_map_rgba_f(1.0f*alpha,1.0f*alpha,1.0f*alpha,alpha);
    alpha = 0.1f;
    ALLEGRO_COLOR infl_col = al_map_rgba_f(1.0f*alpha,1.0f*alpha, 0, alpha);
    ALLEGRO_COLOR quad_grid = al_map_rgba_f(0,1.0f*alpha,0,alpha);

    float view_l = 512;
    float view_x = 10, view_y = 10;

    float cell_l = view_l / float(grid_dim);

    for(int i=0; i<grid_dim; i++) {
        al_draw_line(view_x + i * cell_l, view_y, view_x + i * cell_l, view_y + view_l, quad_grid, 1);
        al_draw_line(view_x, view_y + i * cell_l, view_x + view_l, view_y + i * cell_l, quad_grid, 1);

        for (int j = 0; j < grid_dim; j++)
            grid[i*grid_dim +j] = msg->grid[i*grid_dim + j];
    }

    al_draw_rectangle(view_x, view_y, view_x + view_l, view_y + view_l, quad_col, 1);

    int to_x, to_y, to_x2, to_y2;

    for(int i=0; i<grid_dim; i++) {
        for (int j = 0; j < grid_dim; j++) {
            int val = grid[i*grid_dim +j];
            ALLEGRO_COLOR col;

            if (val == 1) {
                col = obst_col;
            } else if(val == 2) {
                col = infl_col;  
            } else if(val == 3) {
                col = gate_col;  
            } else if (val == 10) {
                col = path_col;
            } else if (val == 100) {
                col = scan_col;
                to_x = j;
                to_y = i;
            } else if (val == 101) {
                col = scan_col;
                to_x2 = j;
                to_y2 = i;
            } else {
                continue;
            }

            al_draw_filled_rectangle(   view_x + cell_l * j, view_y + cell_l * i, view_x + cell_l * (j + 1),
                                        view_y + cell_l * (i + 1), col);
        }
    }
    
    int car_length = (view_l/100)*4 / cell_l;
    int xp = grid_dim/2, yp = grid_dim - car_length;
    //get x and y for start and goal from cells position
    float x_part = view_x + xp*cell_l + cell_l/2,   y_part = view_y + yp*cell_l + cell_l/2;
    float x_goal = view_x + to_x*cell_l + cell_l/2, y_goal = view_y + to_y*cell_l + cell_l/2;
    
    float car_lenght = view_l/100*4;
    float car_width = view_l/100*3;
    al_draw_rectangle(  view_x + cell_l/2 + view_l/2 - car_width/2, view_y + view_l, 
                        view_x + cell_l/2 + view_l/2 + car_width/2, view_y + view_l - car_lenght, 
                        al_map_rgb(255, 255, 0),1);
    al_draw_line(x_part, y_part, x_goal, y_goal, scan_col, 2);

    x_goal = view_x + to_x2*cell_l + cell_l/2, y_goal = view_y + to_y2*cell_l + cell_l/2;
    al_draw_line(x_part, y_part, x_goal, y_goal, scan_col, 2);

    al_draw_textf(font, quad_col, view_x, view_y + view_l, 0, "%f", estimated_speed);
    al_flip_display();
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "dinonav_viewer");

    ros::NodeHandle n;

    ros::Subscriber m_sub = n.subscribe("dinonav/stat", 1,   map_recv);
    ros::Subscriber s_sub = n.subscribe("dinonav/speed", 1, speed_recv);

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
