//
// Created by qixiaofeng on 2020/8/3.
//

#include <stdlib.h>

#include "ai-test.h"
#include "testool.h"
#include "board.h"

unsigned
dummy_evaluator(Board const *board, Ray const *ray) {
    (void) board, (void) ray;
    return 1;
}

void
test_point_validator() {
    Board board;
    clear_board(&board);
    Point p = {0, 0};

    cb_point_validator_t pv = p_get_point_validator('o');
    HandDescription toPut = {0, 0, m_first_appearance};
    put_piece_at(&board, &toPut);
    toPut.y = 1, toPut.appearance = m_second_appearance;
    put_piece_at(&board, &toPut);
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.y = 0;

    pv = p_get_point_validator('*');
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.y = 0;

    pv = p_get_point_validator('_');
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.y = 0;
    pv = p_get_point_validator('.');
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.y = 0;


    pv = p_get_point_validator('|');
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.y = 0;

    pv = p_get_point_validator(':');
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.y = 0;

    pv = p_get_point_validator('x');
    M_test_int(pv(&board, &p, m_first_appearance), false);
    p.x = -1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
    p.x = 0, p.y = 1;
    M_test_int(pv(&board, &p, m_first_appearance), true);
}

void
test_miscs() {
    wchar_t const toUse = L'D';
    ai_set_appearance(toUse);
    M_test_int(ai_get_appearance(), toUse);

    Board board;
    clear_board(&board);
    Point p = {
            .x = 0,
            .y = 0
    };
    p_print_point(&p);
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            p.x = i;
            p.y = j;
            M_test_int(p_ai_board_get(&board, &p), m_empty_appearance);
        }
    }

    M_test_int(p_validate_patterns(), -1);

    unsigned toSort[] = {5, 4, 3, 8, 7, 6, 2, 1, 0, 9,};
    qsort(toSort, 10, sizeof(unsigned), p_desc_compare_unsigned);
    for (int i = 0; i < 10; ++i) {
        M_test_int(toSort[i], 9 - i);
    }

    PointValues pv = {0, 0, {1, 2, 3, 4,}};
    M_test_int(p_sum_point_values(&pv), 10);

    PointValues pvs[] = {
            {5, 5, {1, 2, 2, 0,}},
            {4, 4, {1, 2, 1, 0,}},
            {7, 7, {1, 2, 3, 1,}},
            {8, 8, {1, 2, 3, 2,}},
            {6, 6, {1, 2, 3, 0,}},
            {2, 2, {1, 1, 0, 0,}},
            {3, 3, {1, 2, 0, 0,}},
            {9, 9, {1, 2, 3, 3,}},
            {0, 0, {0, 0, 0, 0,}},
            {1, 1, {1, 0, 0, 0,}},
    };
    qsort(pvs, 10, sizeof(PointValues), p_desc_compare_point_values);
    for (int i = 0; i < 10; ++i) {
        int targetValue = 9 - i;
        PointValues *ptr = &(pvs[i]);
        M_test_int(p_sum_point_values(ptr), targetValue);
        M_test_int(ptr->y, targetValue);
        M_test_int(ptr->x, targetValue);
    }
}

void
test_position_value() {
    Board board;
    clear_board(&board);
    BoardRegion region = {0, 0, 14, 4};
    Point sourcePoint = {
            .x = 0,
            .y = 0
    };
    PointValues pv = p_evaluate_point(&board, &sourcePoint, m_first_appearance, dummy_evaluator);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv.values[i], 1);
    }
    sourcePoint.x = sourcePoint.y = 2;
    pv = p_evaluate_point(&board, &sourcePoint, m_first_appearance, dummy_evaluator);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv.values[i], 1);
    }

    HandDescription hand = {
            .x = 3,
            .y = 3,
            .appearance = m_second_appearance,
    };
    put_piece_at(&board, &hand);
    sourcePoint.x = sourcePoint.y = 3;
    pv = p_evaluate_point(&board, &sourcePoint, m_first_appearance, p_default_evaluator);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv.values[i], 0);
    }
    sourcePoint.x = 4;
    pv = p_evaluate_point(&board, &sourcePoint, m_second_appearance, p_default_evaluator);
    M_test_int(pv.values[0], 5);
    M_test_int(pv.values[1], 2);
    M_test_int(pv.values[2], 2);
    M_test_int(pv.values[3], 2);
    hand.x = 5;
    put_piece_at(&board, &hand);
    hand.y = 2;
    put_piece_at(&board, &hand);
    pv = p_evaluate_point(&board, &sourcePoint, m_second_appearance, p_default_evaluator);
    M_test_int(pv.values[0], 7);
    M_test_int(pv.values[1], 2);
    M_test_int(pv.values[2], 2);
    M_test_int(pv.values[3], 5);

    print_board_region(&board, &region);
}

void
test_match_pattern() {
    Board board;
    clear_board(&board);
    HandDescription toPut = {0, 1, m_second_appearance};
    BoardRegion region = {0, 0, 9, 3};

    char const pattern_a[] = "\x02|.|\xFF";
    Ray ray = {0, 0, 0, 1, m_first_appearance};
    M_test_int(p_match_pattern(&board, &ray, pattern_a), false);
    put_piece_at(&board, &toPut);
    toPut.x = 2, toPut.y = 2;
    put_piece_at(&board, &toPut);
    M_test_int(p_match_pattern(&board, &ray, pattern_a), true);

    toPut.x = 1, toPut.y = 0;
    put_piece_at(&board, &toPut);
    ray.dx = 1, ray.dy = 0;
    M_test_int(p_match_pattern(&board, &ray, pattern_a), true);
    ray.x = 1, ray.y = 1;
    M_test_int(p_match_pattern(&board, &ray, pattern_a), false);
    toPut.x = 3, toPut.y = 0;
    put_piece_at(&board, &toPut);
    ray.x = 2, ray.y = 0;
    M_test_int(p_match_pattern(&board, &ray, pattern_a), true);

    char const pattern_b[] = "\x02|.____\x02";
    ray.dx = 0, ray.dy = 1;
    M_test_int(p_match_pattern(&board, &ray, pattern_b), false);
    ray.dx = 1, ray.dy = 0, ray.x = 4, ray.y = 0;
    M_test_int(p_match_pattern(&board, &ray, pattern_b), true);
    toPut.x = 8, toPut.y = 0;
    put_piece_at(&board, &toPut);
    M_test_int(p_match_pattern(&board, &ray, pattern_b), false);
    toPut.x = 7, toPut.appearance = m_first_appearance;
    put_piece_at(&board, &toPut);

    char const pattern_c[] = "\x02|.***|\xFF";
    M_test_int(p_match_pattern(&board, &ray, pattern_c), true);
    char const pattern_d[] = "\x02|.*|\xFF";
    ray.x = 1, ray.y = 1, ray.dx = 1, ray.dy = 1;
    M_test_int(p_match_pattern(&board, &ray, pattern_d), true);
    ray.x = 0, ray.y = 0;
    M_test_int(p_match_pattern(&board, &ray, pattern_d), true);
    ray.x = 2, ray.y = 2;
    M_test_int(p_match_pattern(&board, &ray, pattern_d), false);
    char const pattern_e[] = "\x02|.**|\xFF";
    ray.x = 2, ray.y = 1, ray.dx = 1, ray.dy = -1;
    M_test_int(p_match_pattern(&board, &ray, pattern_e), true);

    print_board_region(&board, &region);
}

void
test_board_evaluator() {
    Board board;
    clear_board(&board);
    BoardValues values;

    p_evaluate_board(&board, &values, m_first_appearance);
    PointValues const *pv = &(values.points[0]);
    M_test_int(pv->x, 4);
    M_test_int(pv->y, 4);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv->values[i], 4);
    }

    pv = &(values.points[2]);
    M_test_int(pv->x, 6);
    M_test_int(pv->y, 4);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv->values[i], 4);
    }

    pv = &(values.points[49]);
    M_test_int(pv->x, 4);
    M_test_int(pv->y, 0);
    M_test_int(pv->values[0], 4);
    for (int i = 1; i < 4; ++i) {
        M_test_int(pv->values[i], 2);
    }

    pv = &(values.points[224]);
    M_test_int(pv->x, 14);
    M_test_int(pv->y, 14);
    M_test_int(pv->values[0], 2);
    M_test_int(pv->values[1], 2);
    M_test_int(pv->values[2], 2);
    M_test_int(pv->values[3], 0);

    HandDescription toPut = {7, 7, m_first_appearance};
    put_piece_at(&board, &toPut);
    p_evaluate_board(&board, &values, m_first_appearance);
    pv = &(values.points[224]);
    M_test_int(pv->x, 7);
    M_test_int(pv->y, 7);
    for (int i = 0; i < 4; ++i) {
        M_test_int(pv->values[i], 0);
    }
    pv = &(values.points[0]);
    M_test_int(pv->x, 4);
    M_test_int(pv->y, 4);
    M_test_int(pv->values[0], 5);
    for (int i = 1; i < 4; ++i) {
        M_test_int(pv->values[i], 4);
    }
    for (int i = 8; i < 11; ++i) {
        toPut.x = i;
        put_piece_at(&board, &toPut);
    }
    p_evaluate_board(&board, &values, m_first_appearance);
    pv = &(values.points[0]);
    M_test_int(pv->x, 6);
    M_test_int(pv->y, 7);
    M_test_int(pv->values[0], 10);
    for (int i = 1; i < 4; ++i) {
        M_test_int(pv->values[i], 4);
    }
}

void
test_ai() {
    M_run_test_suite(test_miscs);
    M_run_test_suite(test_match_pattern);
    M_run_test_suite(test_position_value);
    M_run_test_suite(test_point_validator);
    M_run_test_suite(test_board_evaluator);
}
