#include <iostream>
#include <math.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

ALLEGRO_FONT *font;

void map_recv(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("map recived");
    al_clear_to_color(al_map_rgb(0,0,0));

    int GRID_DIM = msg->info.width;

    static int *grid= NULL;
    if(grid == NULL)
        grid = new int[GRID_DIM*GRID_DIM];

    float alpha = 0.3f;
    ALLEGRO_COLOR scan_col = al_map_rgba_f(1.0f, 0, 0, 1.0f);
    ALLEGRO_COLOR obst_col = al_map_rgba_f(1.0f*alpha,1.0f*alpha, 0, alpha);
    ALLEGRO_COLOR quad_col = al_map_rgba_f(0,1.0f,0, 1.0f);
    ALLEGRO_COLOR path_col = al_map_rgba_f(0,1.0f*alpha,1.0f*alpha,alpha);
    alpha = 0.1f;
    ALLEGRO_COLOR quad_grid = al_map_rgba_f(0,1.0f*alpha,0,alpha);

    float view_l = 512;
    float view_x = 10, view_y = 10;

    float cell_l = view_l / float(GRID_DIM);

    for(int i=0; i<GRID_DIM; i++) {
        al_draw_line(view_x + i * cell_l, view_y, view_x + i * cell_l, view_y + view_l, quad_grid, 1);
        al_draw_line(view_x, view_y + i * cell_l, view_x + view_l, view_y + i * cell_l, quad_grid, 1);

        for (int j = 0; j < GRID_DIM; j++)
            grid[i*GRID_DIM +j] = msg->data[i*GRID_DIM + j];
    }

    al_draw_rectangle(view_x, view_y, view_x + view_l, view_y + view_l, quad_col, 1);

    for(int i=0; i<GRID_DIM; i++) {
        for (int j = 0; j < GRID_DIM; j++) {
            if (grid[i*GRID_DIM +j] == 1) {
                al_draw_filled_rectangle(view_x + cell_l * j, view_y + cell_l * i, view_x + cell_l * (j + 1),
                                         view_y + cell_l * (i + 1), obst_col);
            }
            if (grid[i*GRID_DIM +j] == 10) {
                al_draw_filled_rectangle(view_x + cell_l * j, view_y + cell_l * i, view_x + cell_l * (j + 1),
                                         view_y + cell_l * (i + 1), path_col);
            }
        }
    }

    /*
    float x_part = view_x + xp*cell_l + cell_l/2,   y_part = view_y + yp*cell_l + cell_l/2;
    float x_goal = view_x + to_x*cell_l + cell_l/2, y_goal = view_y + to_y*cell_l + cell_l/2;
    al_draw_line(x_part, y_part, x_goal, y_goal, scan_col, 2);
    */
    al_flip_display();
}


int main(int argc, char **argv) {
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "dinonav_viewer");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    ros::Subscriber sub = n.subscribe("dinonav/map", 1, map_recv);
    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */

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
