//
// Created by qixiaofeng on 2020/8/3.
//

#include <wchar.h>

#include "ai-test.h"
#include "testool.h"
#include "macro-constants.h"

int
dummy_evaluator(Point const *const sourcePoint, Point const *const targetPoint) {
    return sourcePoint->x + targetPoint->y;
}

void
test_accessors() {
    wchar_t toUse = L'D';
    ai_set_appearance(toUse);
    M_test_int(ai_get_appearance(), toUse)

    p_clear_ai_board();
    Point p = {
            .x = 0,
            .y = 0
    };
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            p.x = i;
            p.y = j;
            M_test_int(p_ai_board_get(&p), m_empty_point)
        }
    }
}

void
test_position_value() {
    Point sourcePoint = {
            .x = 0,
            .y = 0
    };
    p_evaluate_point(&sourcePoint, dummy_evaluator);
}

void
test_ai() {
    M_run_test_suite(test_position_value)
    M_run_test_suite(test_accessors)
}
