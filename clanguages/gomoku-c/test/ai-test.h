//
// Created by qixiaofeng on 2020/8/3.
//

#ifndef GOMOKU_C_AI_TEST_H
#define GOMOKU_C_AI_TEST_H

#include "ai-player.h"

int p_validate_patterns();

cb_point_validator_t p_get_point_validator(char pattern);

wchar_t p_ai_board_get(Board const *board, Point const *point);

PointValues p_evaluate_point(
        Board const *board, Point const *sourcePoint,
        wchar_t allyAppearance, cb_evaluator_t cbEvaluator
);

unsigned p_default_evaluator(Board const *board, Ray const *);

bool p_match_pattern(Board const *board, Ray const *ray, char const *pattern);

void test_ai();

#endif //GOMOKU_C_AI_TEST_H
