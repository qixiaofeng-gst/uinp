//
// Created by qixiaofeng on 2020/8/6.
//

#ifndef GOMOKU_C_MACRO_FUNCTIONS_H
#define GOMOKU_C_MACRO_FUNCTIONS_H

#include "macro-constants.h"

#define M_clear_board(toClear, initValue) \
for(int i = 0; i < m_table_logic_size; ++i) {\
    for(int j = 0; j < m_table_logic_size; ++j) {\
        toClear[i][j] = initValue;\
    }\
}

#endif //GOMOKU_C_MACRO_FUNCTIONS_H
