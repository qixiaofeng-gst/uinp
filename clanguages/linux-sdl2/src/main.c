#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include "macro-functions.h"
#include "callback-types.h"
#include "render.h"

int const G_screen_width = 640;
int const G_screen_height = 480;
int const G_center_flag = SDL_WINDOWPOS_CENTERED_MASK;

SDL_Window *g_window = NULL;
SDL_Renderer *g_renderer = NULL;
cb_render render = NULL;

bool init_sdl2() {
    bool success = true;

    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        success = false;
    } else {
        render = default_render;
        //Create window
        g_window = SDL_CreateWindow(
                "Canvas2D", G_center_flag, G_center_flag,
                G_screen_width, G_screen_height, SDL_WINDOW_SHOWN
        );
        if (g_window == NULL) {
            printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
            success = false;
        } else {
            //Get window surface
            g_renderer = SDL_CreateRenderer(g_window, -1, SDL_RENDERER_ACCELERATED);
        }
    }

    return success;
}

void close_sdl2() {
    SDL_DestroyRenderer(g_renderer);
    g_renderer = NULL;
    SDL_DestroyWindow(g_window);
    g_window = NULL;

    //Quit SDL subsystems
    SDL_Quit();
}

int main() {
    //Start up SDL and create window
    if (false == init_sdl2()) {
        printf("Failed to initialize!\n");
        goto tag_quit;
    }

    //Main loop flag
    bool isRunning = true;

    //Event handler
    SDL_Event e;

    double const deltaSeconds = 0.01;
    Uint32 const deltaTicks = M_round(deltaSeconds * 1e3);
    Uint32 const startTicks = SDL_GetTicks();

    Uint32 nextTicks = startTicks;
    Uint32 counter = 0;
    while (isRunning) {
        counter++;
        nextTicks += deltaTicks;

        while (SDL_PollEvent(&e)) {
            //User requests quit
            if (e.type == SDL_QUIT) {
                isRunning = false;
            } else if (e.type == SDL_KEYDOWN) {
                if (SDLK_ESCAPE == e.key.keysym.sym) {
                    isRunning = false;
                }
            }
        }

        if (NULL == render) {
            goto tag_quit;
        }

        render(g_renderer, deltaSeconds);
        SDL_RenderPresent(g_renderer);

        SDL_Delay(nextTicks - SDL_GetTicks());

        if (counter == 200) {
            break;
        }
    }

    tag_quit:
    close_sdl2();
    return 0;
}
