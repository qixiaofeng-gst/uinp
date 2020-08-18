/******* Includes and global constants *******/
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

int const qxf_c_ScreenWidth = 640;
int const qxf_c_ScreenHeight = 480;

/******* Method declarations *******/
bool qxf_Init();

void qxf_Close();

/******* Global variables *******/
SDL_Window *qxf_g_Window = NULL;
SDL_Renderer *qxf_g_renderer = NULL;

/******* Method definitions *******/
bool qxf_Init() {
    bool success = true;

    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        success = false;
    } else {
        //Create window
        qxf_g_Window = SDL_CreateWindow(
                "SDL Tutorial", 0, 0,
                qxf_c_ScreenWidth, qxf_c_ScreenHeight, SDL_WINDOW_SHOWN
        );
        if (qxf_g_Window == NULL) {
            printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
            success = false;
        } else {
            //Get window surface
            qxf_g_renderer = SDL_CreateRenderer(qxf_g_Window, -1, SDL_RENDERER_ACCELERATED);
        }
    }

    return success;
}

void qxf_Close() {
    SDL_DestroyRenderer(qxf_g_renderer);
    //Destroy window
    SDL_DestroyWindow(qxf_g_Window);
    qxf_g_Window = NULL;

    //Quit SDL subsystems
    SDL_Quit();
}

int main() {
    //Start up SDL and create window
    if (false == qxf_Init()) {
        printf("Failed to initialize!\n");
    } else {
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

            SDL_SetRenderDrawColor(qxf_g_renderer, 0xff, 0xff, 0xff, 0xff);
            SDL_RenderClear(qxf_g_renderer);

            SDL_SetRenderDrawColor(qxf_g_renderer, 0x00, 0x00, 0xFF, 0xFF);
            SDL_RenderDrawLine(
                    qxf_g_renderer,
                    0, qxf_c_ScreenHeight / 2,
                    qxf_c_ScreenWidth, qxf_c_ScreenHeight / 2
            );
            SDL_RenderPresent(qxf_g_renderer);
            SDL_Delay(10);
        }
    }

    //Free resources and close SDL
    qxf_Close();
    return 0;
}
