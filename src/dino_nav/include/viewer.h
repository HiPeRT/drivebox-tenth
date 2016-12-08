#ifndef VIEWER_H
#define VIEWER_H

#include <allegro5/allegro.h>

#define RGBA(r, g, b, a)  al_map_rgba_f(r*a,g*a,b*a, a)

const ALLEGRO_COLOR VIEW_COLOR      = RGBA(0, 1, 0, 1.0);
const ALLEGRO_COLOR VIEW_GRID_COLOR = RGBA(0, 1, 0, 0.1);
const ALLEGRO_COLOR WALL_COLOR      = RGBA(0, 1, 1, 0.5);
const ALLEGRO_COLOR INFLATED_COLOR  = RGBA(0, 1, 1, 0.2);
const ALLEGRO_COLOR GATE_COLOR      = RGBA(1, 1, 1, 0.2);
const ALLEGRO_COLOR PATH_COLOR      = RGBA(1, 0, 0, 1.0);
const ALLEGRO_COLOR PATH_GRID_COLOR = RGBA(1, 0, 1, 0.2);
const ALLEGRO_COLOR CAR_COLOR       = RGBA(0, 1, 1, 1.0);

#endif //VIEWER_H
