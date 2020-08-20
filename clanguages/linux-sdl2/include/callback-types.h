//
// Created by qixiaofeng on 2020/8/20.
//

#ifndef QXF_CANVAS_CALLBACK_TYPES_H
#define QXF_CANVAS_CALLBACK_TYPES_H

struct SDL_Renderer;
typedef struct SDL_Renderer SDL_Renderer;

typedef void (* cb_render) (SDL_Renderer *renderer, double deltaSeconds);

#endif //QXF_CANVAS_CALLBACK_TYPES_H
