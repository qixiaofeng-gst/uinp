//
// Created by qixiaofeng on 2020/8/20.
//
#include <SDL2/SDL.h>
#include "render.h"
#include "macro-functions.h"
#include "physics.h"

typedef void (*p_cb_shader)(SDL_Renderer *renderer, int x, int y, int nx, int ny);

/*!
 * x^2 + y^2 = r^2, <br>
 * y = (r^2 - x^2)^{1/2}
 */
void p_shape_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius, p_cb_shader cbShader) {
    static double const side_ratio = 1 / 1.414;

    int const limit = M_round(side_ratio * radius) + 1;
    int const radius_sq2 = radius * radius;
    for (int vX = 0; vX < limit; ++vX) {
        int const vY = M_round(SDL_sqrt(radius_sq2 - vX * vX));
        cbShader(renderer, oX + vX, oY + vY, oX - vX, oY - vY);
        cbShader(renderer, oX + vY, oY + vX, oX - vY, oY - vX);
    }
}

void p_circle_outline_shader(SDL_Renderer *renderer, int x, int y, int nx, int ny) {
    SDL_RenderDrawPoint(renderer, x, y);
    SDL_RenderDrawPoint(renderer, nx, y);
    SDL_RenderDrawPoint(renderer, nx, ny);
    SDL_RenderDrawPoint(renderer, x, ny);
}

void p_draw_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius) {
    p_shape_circle_with_formula(renderer, oX, oY, radius, p_circle_outline_shader);
}

void p_circle_solid_shader(SDL_Renderer *renderer, int x, int y, int nx, int ny) {
    SDL_RenderDrawLine(renderer, x, y, x, ny);
    SDL_RenderDrawLine(renderer, nx, y, nx, ny);
}

void p_fill_circle_with_formula(SDL_Renderer *renderer, int oX, int oY, int radius) {
    p_shape_circle_with_formula(renderer, oX, oY, radius, p_circle_solid_shader);
}

void p_draw_rotating_line(SDL_Renderer *renderer, int oX, int oY, double theta) {
    int const length = 50;
    int const x = M_round(SDL_sin(theta) * length);
    int const y = M_round(SDL_cos(theta) * length);
    SDL_RenderDrawLine(renderer, oX, oY, x + oX, y + oY);
}

void p_draw_circle(SDL_Renderer *renderer, Circle *circle) {
    p_fill_circle_with_formula(
            renderer,
            M_round(circle->origin.x),
            M_round(circle->origin.y),
            M_round(circle->radius)
    );
}

void default_render(SDL_Renderer *renderer, double deltaSeconds) {
    static double passedSeconds = 0.0;
    static RigidCircle rigidCircle = {
            .circle = {
                    .origin = {
                            .x = 10.0,
                            .y = 10.0,
                    },
                    .radius = 10.0,
            },
            .motion = {
                    .velocity = {
                            .x = 0.0,
                            .y = 0.0,
                    },
                    .acceleration = {
                            .x = 0.0,
                            .y = 0.0,
                    },
            },
    };
    if (passedSeconds == 0.0) {
        add_gravity_to(&rigidCircle);
    }

    passedSeconds += deltaSeconds;
    double const theta = 3.141592 * passedSeconds;

    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xFF, 0xFF);
    p_draw_circle_with_formula(renderer, 50, 50, 50);
    p_draw_circle_with_formula(renderer, 150, 50, 50);
    p_draw_circle_with_formula(renderer, 210, 210, 200);
    p_fill_circle_with_formula(renderer, 50, 50, 10);
    p_draw_rotating_line(renderer, 100, 50, theta);

    update_motion(&rigidCircle, deltaSeconds);
    p_draw_circle(renderer, &rigidCircle.circle);
}
