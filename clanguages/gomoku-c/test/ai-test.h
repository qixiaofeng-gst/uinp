//
// Created by qixiaofeng on 2020/8/3.
//

#ifndef GOMOKU_C_AI_TEST_H
#define GOMOKU_C_AI_TEST_H

#include "ai-player.h"

wchar_t p_ai_board_get(Point const *point);

int p_evaluate_point(Point const *sourcePoint, cb_evaluator_t cbEvaluator);

void p_clear_ai_board();

void test_ai();

#endif //GOMOKU_C_AI_TEST_H
