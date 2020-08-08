//
// Created by qixiaofeng on 2020/8/3.
//

#include <wchar.h>

#include "ai-test.h"
#include "testool.h"
#include "board.h"

unsigned
dummy_evaluator(Board const *board, Point const *sourcePoint, Point const *targetPoint) {
    if (m_empty_appeance == board->grids[sourcePoint->x][sourcePoint->y]) {
        return sourcePoint->x + targetPoint->y;
    }
    return 1;
}

void
test_miscs() {
    wchar_t const toUse = L'D';
    ai_set_appearance(toUse);
    M_test_int(ai_get_appearance(), toUse)

    Board board;
    clear_board(&board);
    Point p = {
            .x = 0,
            .y = 0
    };
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            p.x = i;
            p.y = j;
            M_test_int(p_ai_board_get(&board, &p), m_empty_appeance)
        }
    }
}

void
test_point_validator() {
    Board board;
    clear_board(&board);
    Point p = {0, 0};
    M_test_int(p_validate_patterns(), -1)
    cb_point_validator_t pv = p_get_point_validator('o');
    M_test_int(pv(&board, &p, m_empty_appeance), true)
}

void
test_position_value() {
    Board board;
    clear_board(&board);
    Point sourcePoint = {
            .x = 0,
            .y = 0
    };
    M_test_int(p_evaluate_point(&board, &sourcePoint, dummy_evaluator), 0)
    sourcePoint.x = sourcePoint.y = 2;
    M_test_int(p_evaluate_point(&board, &sourcePoint, dummy_evaluator), 4)
    M_test_int(p_evaluate_point(&board, &sourcePoint, p_default_evaluator), 1)
    HandDescription hand = {
            .x = 3,
            .y = 3,
            .appearance = m_first_appearance,
    };
    put_piece_at(&board, &hand);
    sourcePoint.x = sourcePoint.y = 3;
    M_test_int(p_evaluate_point(&board, &sourcePoint, p_default_evaluator), 0)
}

void
test_ai() {
    M_run_test_suite(test_position_value)
    M_run_test_suite(test_miscs)
    M_run_test_suite(test_point_validator)
}
