//
// Created by qixiaofeng on 2020/8/20.
//
#include <SDL2/SDL.h>
#include "render.h"
#include "macro-functions.h"

typedef void (* p_cb_draw_circle)(SDL_Renderer *renderer, int x, int y, int nx, int ny);

/*!
 * x^2 + y^2 = r^2, <br>
 * y = (r^2 - x^2)^{1/2}
 */
void p_shape_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius, p_cb_draw_circle cbDraw) {
    static double const side_ratio = 1 / 1.414;

    int const limit = M_round(side_ratio * radius) + 1;
    int const radius_sq2 = radius * radius;
    for (int vX = 0; vX < limit; ++vX) {
        int const vY = M_round(SDL_sqrt(radius_sq2 - vX * vX));
        int const x = oX + vX;
        int const nx = oX - vX;
        int const y = oY + vY;
        int const ny = oY - vY;
        cbDraw(renderer, x, y, nx, ny);
        cbDraw(renderer, y, x, ny, nx);
    }
}

void p_draw_circle_outline(SDL_Renderer *renderer, int x, int y, int nx, int ny) {
    SDL_RenderDrawPoint(renderer, x, y);
    SDL_RenderDrawPoint(renderer, nx, y);
    SDL_RenderDrawPoint(renderer, nx, ny);
    SDL_RenderDrawPoint(renderer, x, ny);
}

void draw_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius) {
    p_shape_circle_with_formula(renderer, oX, oY, radius, p_draw_circle_outline);
}

void p_draw_circle_solid(SDL_Renderer *renderer, int x, int y, int nx, int ny) {
    SDL_RenderDrawLine(renderer, x, y, x, ny);
    SDL_RenderDrawLine(renderer, nx, y, nx, ny);
}

void fill_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius) {
    p_shape_circle_with_formula(renderer, oX, oY, radius, p_draw_circle_solid);
}

void draw_rotating_line(SDL_Renderer *renderer, int oX, int oY, double theta) {
    int const length = 50;
    int const x = M_round(SDL_sin(theta) * length);
    int const y = M_round(SDL_cos(theta) * length);
    SDL_RenderDrawLine(renderer, oX, oY, x + oX, y + oY);
}

void p_default_render(SDL_Renderer *renderer, double deltaSeconds) {
    static double passedSeconds = 0.0;

    passedSeconds += deltaSeconds;
    double const theta = 3.141592 * passedSeconds;

    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xFF, 0xFF);
    draw_circle_with_formula(renderer, 50, 50, 50);
    fill_circle_with_formula(renderer, 50, 50, 10);
    draw_rotating_line(renderer, 100, 50, theta);
}
