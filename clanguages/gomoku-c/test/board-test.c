#include "testool.h"
#include "board.h"
#include "board-test.h"
#include "macro-constants.h"

void
test_board_edge_checkers() {
    M_test_int(p_check_bottom_border(14), true)
    M_test_int(p_check_bottom_border(13), true)
    M_test_int(p_check_bottom_border(15), false)

    M_test_int(p_check_up_border(-1), false)
    M_test_int(p_check_up_border(0), true)
    M_test_int(p_check_up_border(1), true)
}

void
test_board_point_validator() {
    Point p = {0, 0};
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            p.x = i, p.y = j;
            M_test_int(validate_board_point(&p), true)
        }
    }
    p.x = m_table_logic_size, p.y = m_table_logic_size;
    M_test_int(validate_board_point(&p), false)
    p.y = 0, p.x = 0;
    M_test_int(validate_board_point(&p), true)
    p.x = m_table_logic_size, p.y = m_table_logic_size;
    M_test_int(validate_board_point(&p), false)
}

void
test_continuous_counter() {
    Board board;
    HandDescription hand = {
            .x = 0,
            .y = 0,
            .appearance = m_first_appearance
    };
    int const flagOne = m_first_appearance;
    int const flagTwo = m_second_appearance;
    clear_board(&board);
    for (int i = 0; i < 4; ++i) {
        hand.y = i;
        put_piece_at(&board, &hand);
    }
    hand.y = 3;
    M_test_int(is_game_end(&board, &hand), false)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 0, 3, 0, -1), 3)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 0, 1, 0, -1), 1)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 0, 1, 0, 1), 2)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 0, 0, 0, 1), 3)
    hand.y = 1;
    hand.appearance = flagTwo;
    put_piece_at(&board, &hand);
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, 0, 1), 0)
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, 0, -1), 0)
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, 1, -1), 0)
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, 1, 1), 0)
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, 1, 0), 0)
    M_test_int(p_count_continuous_same_flag(&board, flagTwo, 0, 1, -1, 0), 0)
    M_test_int(p_count_continuous_same_flag(&board, m_empty_appeance, 0, 5, 0, -1), 1)
    M_test_int(p_count_continuous_same_flag(&board, m_empty_appeance, 0, 5, 0, 1), 9)

    clear_board(&board);
    hand.appearance = flagOne;
    for (int i = 0; i < 9; ++i) {
        hand.x = hand.y = i;
        put_piece_at(&board, &hand);
    }
    hand.appearance = flagTwo;
    hand.x = hand.y = 9;
    put_piece_at(&board, &hand);

    hand.appearance = flagOne;
    hand.x = hand.y = 3;
    M_test_int(is_game_end(&board, &hand), true)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 1, 1, -1, -1), 1)
    M_test_int(p_count_continuous_same_flag(&board, flagOne, 1, 1, 1, 1), 7)
    for (int i = 0; i < 9; ++i) {
        hand.x = hand.y = i;
        M_test_int(is_game_end(&board, &hand), true)
    }
    Point point = {.x = 10, .y = 10};
    M_test_int(is_empty_slot(&board, &point), true)
    hand.x = hand.y = 9;
    M_test_int(is_on_board(&board, &hand), false)
    clear_board(&board);
    for (int i = 0; i < 9; ++i) {
        int j = 8 - i;
        hand.x = i;
        hand.y = j;
        put_piece_at(&board, &hand);
    }
    for (int i = 0; i < 9; ++i) {
        int j = 8 - i;
        hand.x = i;
        hand.y = j;
        M_test_int(is_game_end(&board, &hand), true)
    }
    hand.x = hand.y = 9;
    M_test_int(is_game_end(&board, &hand), false)
}

void
test_board_checkers() {
    M_run_test_suite(test_board_edge_checkers)
    M_run_test_suite(test_continuous_counter)
    M_run_test_suite(test_board_point_validator)
}
