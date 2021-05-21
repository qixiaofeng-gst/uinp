//
// Created by qixiaofeng on 2021/4/7.
//
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
// sudo apt install -y libglew-dev
#include <GL/glew.h>
#include <SDL2/SDL.h>

#define WinWidth 1000
#define WinHeight 1000

static uint32_t const g_base_flag = SDL_WINDOW_OPENGL;

void toggle_full_screen(SDL_Window *window) {
    static uint32_t const full_screen_flag = SDL_WINDOW_FULLSCREEN_DESKTOP;
    static uint32_t const flags_with_full_screen = g_base_flag | full_screen_flag;

    static bool is_full_screen = false;

    if (is_full_screen) {
        SDL_SetWindowFullscreen(window, g_base_flag);
        is_full_screen = false;
    } else {
        SDL_SetWindowFullscreen(window, flags_with_full_screen);
        is_full_screen = true;
    }
}

int main() {
    SDL_Window *window = SDL_CreateWindow("OpenGL Test", 0, 0, WinWidth, WinHeight, g_base_flag);
    assert(window);
    SDL_GLContext context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, context);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    char const *gl_version = (char *) glGetString(GL_VERSION);
    printf("OpenGL Version: %s\n", gl_version);
    // glCreateProgram();

    bool Running = true;
    while (Running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        Running = false;
                        break;
                    case 'f':
                        toggle_full_screen(window);
                        break;
                    default:
                        break;
                }
            } else if (event.type == SDL_QUIT) {
                Running = false;
            }
        }

        glViewport(0, 0, WinWidth, WinHeight);
        glClearColor(0.f, 0.f, 0.f, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);

        SDL_GL_SwapWindow(window);
    }
    return 0;
}
