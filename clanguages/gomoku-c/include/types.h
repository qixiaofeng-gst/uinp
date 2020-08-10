//
// Created by qixiaofeng on 2020/7/30.
//

#ifndef GOMOKU_C_TYPES_H
#define GOMOKU_C_TYPES_H

#include <wchar.h>
#include <stdbool.h>

#include "macro-constants.h"

typedef struct p_Board {
    wchar_t grids[m_table_logic_size][m_table_logic_size];
} Board;

typedef struct p_BoardRegion {
    size_t x;
    size_t y;
    size_t w;
    size_t h;
} BoardRegion;

typedef struct p_PointValues {
    unsigned values[4];
} PointValues;

typedef struct p_Point {
    int x;
    int y;
} Point;

typedef struct p_Ray {
    int x;
    int y;
    int dx;
    int dy;
    wchar_t appearance;
} Ray;

typedef struct {
    int x;
    int y;
    wchar_t appearance;
} HandDescription;

typedef void (*cb_player_t)(Board *, HandDescription const *, HandDescription *);

typedef bool (*cb_point_validator_t)(Board const *, Point const *, wchar_t);

typedef unsigned (*cb_evaluator_t)(Board const *, Ray const *);

#endif //GOMOKU_C_TYPES_H
