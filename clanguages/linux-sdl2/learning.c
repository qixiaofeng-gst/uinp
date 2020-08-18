/******* Includes and global constants *******/
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

enum qxf_enum_SurfaceIndex {
    SurfaceIndexDefault,
    SurfaceIndexUp,
    SurfaceIndexDown,
    SurfaceIndexRight,
    SurfaceIndexLeft,
    SurfaceIndexLimit,
};

const int qxf_c_ScreenWidth = 640;
const int qxf_c_ScreenHeight = 480;

/******* Method declarations *******/
bool qxf_Init();

void qxf_Close();

/******* Global variables *******/
SDL_Window *qxf_g_Window = NULL;
SDL_Surface *qxf_g_ScreenSurface = NULL;

SDL_Surface *qxf_g_CurrentImageSurface = NULL;
SDL_Surface *qxf_g_PreloadedImageSurface[SurfaceIndexLimit];

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
            qxf_g_ScreenSurface = SDL_GetWindowSurface(qxf_g_Window);
        }
    }

    return success;
}

void qxf_Close() {
    //Deallocate surface
    for (int i = 0; i < SurfaceIndexLimit; ++i) {
        SDL_FreeSurface(qxf_g_PreloadedImageSurface[i]);
        qxf_g_PreloadedImageSurface[i] = NULL;
    }

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
        qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexDefault];

        //Event handler
        SDL_Event e;

        while (isRunning) {
            while (SDL_PollEvent(&e)) {
                //User requests quit
                if (e.type == SDL_QUIT) {
                    isRunning = false;
                } else if (e.type == SDL_KEYDOWN) {
                    switch (e.key.keysym.sym) {
                        case SDLK_UP:
                            qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexUp];
                            break;
                        case SDLK_DOWN:
                            qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexDown];
                            break;
                        case SDLK_RIGHT:
                            qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexRight];
                            break;
                        case SDLK_LEFT:
                            qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexLeft];
                            break;
                        case SDLK_ESCAPE:
                            isRunning = false;
                            break;
                        default:
                            qxf_g_CurrentImageSurface = qxf_g_PreloadedImageSurface[SurfaceIndexDefault];
                    }
                }
            }
            SDL_Rect stretchRect;
            stretchRect.x = 0;
            stretchRect.y = 0;
            stretchRect.w = qxf_c_ScreenWidth;
            stretchRect.h = qxf_c_ScreenHeight;
            SDL_BlitScaled(qxf_g_CurrentImageSurface, NULL, qxf_g_ScreenSurface, &stretchRect);
            SDL_UpdateWindowSurface(qxf_g_Window);
        }
    }

    //Free resources and close SDL
    qxf_Close();
    return 0;
}