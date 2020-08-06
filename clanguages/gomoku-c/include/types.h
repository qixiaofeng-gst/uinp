//
// Created by qixiaofeng on 2020/7/30.
//

#ifndef GOMOKU_C_TYPES_H
#define GOMOKU_C_TYPES_H

#include <wchar.h>

#include "macro-constants.h"

typedef struct p_Board {
    wchar_t grids[m_table_logic_size][m_table_logic_size];
} Board;

typedef struct p_Point {
    int x;
    int y;
} Point;

typedef struct {
    int x;
    int y;
    wchar_t appearance;
} HandDescription;

typedef void (*cb_player_t)(Board *, HandDescription const *, HandDescription *);

typedef unsigned (*cb_evaluator_t)(Board const *, Point const *, Point const *);

#endif //GOMOKU_C_TYPES_H
