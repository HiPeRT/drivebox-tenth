#ifndef VIZ_H
#define VIZ_H

#include <allegro5/allegro.h>
#include "common.h"

#define RGBA(r, g, b, a)  al_map_rgba_f(r*a,g*a,b*a, a)

const ALLEGRO_COLOR VIEW_COLOR      = RGBA(0, 1, 0, 1.0);
const ALLEGRO_COLOR VIEW_GRID_COLOR = RGBA(0, 1, 0, 0.1);
const ALLEGRO_COLOR WALL_COLOR      = RGBA(0, 1, 1, 0.5);
const ALLEGRO_COLOR INFLATED_COLOR  = RGBA(0, 1, 1, 0.2);
const ALLEGRO_COLOR GATE_COLOR      = RGBA(1, 1, 1, 0.2);
const ALLEGRO_COLOR PATH_COLOR      = RGBA(1, 0, 0, 1.0);
const ALLEGRO_COLOR PATH_GRID_COLOR = RGBA(1, 0, 1, 0.2);
const ALLEGRO_COLOR CAR_COLOR       = RGBA(0, 1, 1, 1.0);

bool viz_init();
bool viz_update();
void viz_destroy();
void viz_flip();
void viz_clear();

void viz_rect(float_point_t o, float w, float h, ALLEGRO_COLOR col, float thick);

#endif //VIZ_H

