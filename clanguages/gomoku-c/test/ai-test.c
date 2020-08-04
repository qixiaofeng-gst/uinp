//
// Created by qixiaofeng on 2020/8/3.
//

#include "wchar.h"

#include "ai-test.h"
#include "testool.h"

void
test_accessors() {
    wchar_t toUse = L'D';
    ai_set_appearance(toUse);
    M_test_int(ai_get_appearance(), toUse)
}

void
test_position_value() {
    Point sourcePoint = {
        .x = 0,
        .y = 0
    };
    p_evaluate_point(&sourcePoint);
}

void
test_ai() {
    M_run_test_suite(test_position_value)
    M_run_test_suite(test_accessors)
}
