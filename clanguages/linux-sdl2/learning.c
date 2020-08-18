/******* Includes and global constants *******/
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>

int const qxf_c_ScreenWidth = 640;
int const qxf_c_ScreenHeight = 480;

/*!
var c = document.getElementById('canv');
var $ = c.getContext('2d');
document.body.clientWidth;
var wh = 128;
var w2h = wh * wh;
c.width = c.height =  wh;
var img = $.createImageData(wh, wh);
var id = img.data;
var t = 0;
var inc = 1 / wh;
var arr = [];

for(var k = 0; k < w2h; ++k)
   arr[k] = Math.random() * 1.5 - 0.5;

function draw(){
  window.requestAnimationFrame(draw);
   t += inc;
   for(var x = 1; x >= 0; x -= inc) {
      for(var y = 1; y >= 0; y -= inc) {
         var idx = (y * wh + x) * wh * 4;
         var dx = x;
         var dy = y;
         var dist = Math.sqrt(dx * dx + dy * dy);
         var ax = oct(x, y);
         var ay = oct(x + 2, y + t / 3);
         var bx = oct(x + dist * .3 + ax / 22 + 0.7, y + ay / 5 + 2);
         var by = oct(x + ax / 3 + 4 * t, y + ay / 3 + 5);
         var n = oct(x + bx / 5, y + by / 2) * 0.7 + .15;
         var d = ax * by / 2;
         var e = ay * bx / 2;

         id[idx + 0] = hue(n + d / 5);
         id[idx + 1] = hue(n / 3 + e / 5 + d);
         id[idx + 2] = hue(d + e);
         id[idx + 3] = hue(1 - ease(dist) * (e + d) * 5)
      }
   }
   $.putImageData(img, 0, 0);
}
function hue($) {
   return 255 * Math.min(Math.max($, 0), 1);
}
function ease(x) {
   return (x > 0.2) ? 0 : i(1, 0, x * 6);
}
var db = document.body;
function i($, db, t) {
   t = t * t * t * (6 * t * t - 15 * t + 10);
   return $ + (db - $) * t;
}
function n(x, y) {
   var i = Math.abs(x * wh + y) % w2h;
   return arr[i];
}
function oct(x, y) {
   var o1 = p(x * 3.0, y * 4.0);
   var o2 = p(x * 4.0, y * 5.0);
   return o1 + o2 * 0.5;
}
function p(x, y) {
   var nx = Math.floor(x);
   var ny = Math.floor(y);
   return i(i(n(nx, ny), n(nx + 1, ny), x - nx), i(n(nx, ny + 1), n(nx + 1, ny + 1), x - nx), y - ny);
}
draw();
*/

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
