/******* Includes and global constants *******/
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

#define M_round(x) (int) SDL_floor(0.5 + x)

typedef void (* cb_draw_circle)(int x, int y, int nx, int ny);

int const G_screen_width = 640;
int const G_screen_height = 480;
int const G_center_flag = SDL_WINDOWPOS_CENTERED_MASK;

SDL_Window *g_window = NULL;
SDL_Renderer *g_renderer = NULL;

bool init_sdl2() {
    bool success = true;

    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
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

/*!
 * x^2 + y^2 = r^2, <br>
 * y = (r^2 - x^2)^{1/2}
 */
void p_shape_circle_with_formula(int oX, int oY, int radius, cb_draw_circle cbDraw) {
    static double const side_ratio = 1 / 1.414;

    int const limit = M_round(side_ratio * radius) + 1;
    int const radius_sq2 = radius * radius;
    for (int vX = 0; vX < limit; ++vX) {
        int const vY = M_round(SDL_sqrt(radius_sq2 - vX * vX));
        int const x = oX + vX;
        int const nx = oX - vX;
        int const y = oY + vY;
        int const ny = oY - vY;
        cbDraw(x, y, nx, ny);
        cbDraw(y, x, ny, nx);
    }
}

void p_draw_circle_outline(int x, int y, int nx, int ny) {
    SDL_RenderDrawPoint(g_renderer, x, y);
    SDL_RenderDrawPoint(g_renderer, nx, y);
    SDL_RenderDrawPoint(g_renderer, nx, ny);
    SDL_RenderDrawPoint(g_renderer, x, ny);
}

void draw_circle_with_formula(int oX, int oY, int radius) {
    p_shape_circle_with_formula(oX, oY, radius, p_draw_circle_outline);
}

void p_draw_circle_solid(int x, int y, int nx, int ny) {
    SDL_RenderDrawLine(g_renderer, x, y, x, ny);
    SDL_RenderDrawLine(g_renderer, nx, y, nx, ny);
}

void fill_circle_with_formula(int oX, int oY, int radius) {
    p_shape_circle_with_formula(oX, oY, radius, p_draw_circle_solid);
}

void draw_rotating_line(int oX, int oY, double theta) {
    int const length = 50;
    int const x = M_round(SDL_sin(theta) * length);
    int const y = M_round(SDL_cos(theta) * length);
    SDL_RenderDrawLine(g_renderer, oX, oY, x + oX, y + oY);
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

    double const dTheta = 0.03;
    Uint32 const deltaTicks = 10;
    Uint32 const startTicks = SDL_GetTicks();

    double theta = 0;
    Uint32 nextTicks = startTicks;
    while (isRunning) {
        nextTicks += deltaTicks;
        theta += dTheta;

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
        draw_circle_with_formula(50, 50, 50);
        fill_circle_with_formula(50, 50, 10);
        draw_rotating_line(100, 50, theta);

        SDL_RenderPresent(g_renderer);

        SDL_Delay(nextTicks - SDL_GetTicks());
    }

    tag_quit:
    close_sdl2();
    return 0;
}
