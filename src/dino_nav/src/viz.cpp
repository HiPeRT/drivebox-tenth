#include <stdio.h>
#include <math.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "viz.h"

ALLEGRO_DISPLAY *display;
ALLEGRO_EVENT_QUEUE *event_queue;
ALLEGRO_TIMER *timer;
const int FONTS = 32;
ALLEGRO_FONT *font[FONTS];

ALLEGRO_MOUSE_STATE mouse;

bool viz_init() {  
    if(!al_init()) {
        printf("failed to initialize allegro!\n");
        return false;
    }

    display = al_create_display(840, 600);
    if(!display) {
        printf("failed to create display!\n");
        return false;
    }

    al_install_keyboard();
    al_install_mouse();

    al_init_primitives_addon();
    al_init_font_addon(); // initialize the font addon
    al_init_ttf_addon();// initialize the ttf (True Type Font) addon

    for(int i=0; i<32; i++) {
        font[i] = al_load_ttf_font("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 5+i,0 );
    }

    if (!font){
        printf("Could not load font.\n");
        return false;
    }

    timer = al_create_timer(1.0 / 60);
    if(!timer) {
        printf("failed to create timer!\n");
        return false;
    }

    event_queue = al_create_event_queue();
    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_register_event_source(event_queue, al_get_keyboard_event_source());
    al_register_event_source(event_queue, al_get_timer_event_source(timer));

    al_start_timer(timer);
}

bool viz_update() {
    static bool redraw = false;
    ALLEGRO_EVENT ev;
    al_wait_for_event(event_queue, &ev);

    if(ev.type == ALLEGRO_EVENT_TIMER) {
        redraw = true;
    } else if(ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
        return false;
    } 

    if(redraw && al_event_queue_is_empty(event_queue)) {
        redraw = false;
        al_get_mouse_state(&mouse);
    }
    return true;
}

void viz_destroy() {
    al_destroy_timer(timer);
    al_destroy_display(display);
    al_destroy_event_queue(event_queue);
}

void viz_rect(float_point_t o, float w, float h, ALLEGRO_COLOR col, float thick) {
    if(thick <= 0)
        al_draw_filled_rectangle(o.x, o.y, o.x + w, o.y + h, col);
    else
        al_draw_rectangle(o.x, o.y, o.x + w, o.y + h, col, thick);
}

void viz_circle(float_point_t p, float r, ALLEGRO_COLOR col, float thick) {
    if(thick <= 0)
        al_draw_filled_circle(p.x, p.y, r, col);
    else
        al_draw_circle(p.x, p.y, r, col, thick);
}

void viz_line(float_point_t a, float_point_t b, ALLEGRO_COLOR col, float thick) {
    al_draw_line(a.x, a.y, b.x, b.y, col, thick);
}


void viz_arc(float cx, float cy, float r, float start_theta, float delta_theta, 
             ALLEGRO_COLOR col, float thick) {

    al_draw_arc(cx, cy, r, start_theta, delta_theta, col, thick);
}

void viz_triangle(float_point_t a, float_point_t b, float_point_t c, ALLEGRO_COLOR col, float thick) {

    if(thick <= 0)
        al_draw_filled_triangle(a.x, a.y, b.x, b.y, c.x, c.y, col);
    else    
        al_draw_triangle(a.x, a.y, b.x, b.y, c.x, c.y, col, thick);
}

void viz_text(float x, float y, int dim, ALLEGRO_COLOR col, const char *format, ...) {

    int font_id = dim-5;
    if(font_id <0) font_id=0;
    if(font_id >FONTS-1) font_id=FONTS-1;

    char buf[512];
    va_list va;
    va_start (va, format);
    vsprintf (buf, format, va);
    va_end (va);

    al_draw_text(font[font_id], col, x, y, 0, buf);
}

void viz_clear() {
    al_clear_to_color(al_map_rgb(0,0,0));
}

void viz_flip() {
    al_flip_display();
}