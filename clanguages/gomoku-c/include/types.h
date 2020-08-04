//
// Created by qixiaofeng on 2020/7/30.
//

#ifndef GOMOKU_C_TYPES_H
#define GOMOKU_C_TYPES_H

typedef struct p_Point {
    int x;
    int y;
} Point;

typedef struct {
    int x;
    int y;
    wchar_t appearance;
} HandDescription;

typedef void (*cb_player_t)(HandDescription const *const, HandDescription *const);

#endif //GOMOKU_C_TYPES_H
