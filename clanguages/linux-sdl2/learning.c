/******* Includes and global constants *******/
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

int const G_screen_width = 640;
int const G_screen_height = 480;
int const G_center_flag = 0x2FFF0000; /// Check SDL_WINDOWPOS_CENTERED

/******* Global variables *******/
SDL_Window *g_window = NULL;
SDL_Renderer *g_renderer = NULL;

/******* Method definitions *******/
bool init_sdl2() {
    bool success = true;

    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        success = false;
    } else {
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
    //Destroy window
    SDL_DestroyWindow(g_window);
    g_window = NULL;

    //Quit SDL subsystems
    SDL_Quit();
}

void draw_circle() {
    SDL_RenderDrawPoint(g_renderer, 0, 0);
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

    while (isRunning) {
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

        SDL_SetRenderDrawColor(g_renderer, 0xff, 0xff, 0xff, 0xff);
        SDL_RenderClear(g_renderer);

        SDL_SetRenderDrawColor(g_renderer, 0x00, 0x00, 0xFF, 0xFF);
        SDL_RenderDrawLine(
                g_renderer,
                0, G_screen_height / 2,
                G_screen_width, G_screen_height / 2
        );
        draw_circle();

        SDL_RenderPresent(g_renderer);
        SDL_Delay(10);
    }

    tag_quit:
    close_sdl2();
    return 0;
}
