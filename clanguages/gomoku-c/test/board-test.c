#include "testool.h"
#include "board-test.h"
#include "macro-constants.h"

void
test_board_edge_checkers()
{
    M_test_int(check_bottom_border(14), true)
    M_test_int(check_bottom_border(13), true)
    M_test_int(check_bottom_border(15), false)

    M_test_int(check_up_border(-1), false)
    M_test_int(check_up_border(0), true)
    M_test_int(check_up_border(1), true)
}

void
test_continuous_counter()
{
    int const flagOne = 9;
    int const flagTwo = 3;
    clear_board();
    for (int i = 0; i < 4; ++i) {
        put_piece_at(0, i, flagOne);
    }
    M_test_int(is_game_end(0, 3, flagOne), false)
    M_test_int(count_continuous_same_flag(flagOne, 0, 3, 0, -1), 3)
    M_test_int(count_continuous_same_flag(flagOne, 0, 1, 0, -1), 1)
    M_test_int(count_continuous_same_flag(flagOne, 0, 1, 0, 1), 2)
    M_test_int(count_continuous_same_flag(flagOne, 0, 0, 0, 1), 3)
    put_piece_at(0, 1, flagTwo);
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, 0, 1), 0)
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, 0, -1), 0)
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, 1, -1), 0)
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, 1, 1), 0)
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, 1, 0), 0)
    M_test_int(count_continuous_same_flag(flagTwo, 0, 1, -1, 0), 0)
    M_test_int(count_continuous_same_flag(m_empty_slot, 0, 5, 0, -1), 1)
    M_test_int(count_continuous_same_flag(m_empty_slot, 0, 5, 0, 1), 9)

    clear_board();
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            put_piece_at(i, j, flagOne);
        }
    }
    put_piece_at(9, 9, flagTwo);
    M_test_int(is_game_end(3, 3, flagOne), true)
    M_test_int(count_continuous_same_flag(flagOne, 1, 1, -1, -1), 1)
    M_test_int(count_continuous_same_flag(flagOne, 1, 1, 1, 1), 7)
}

void
test_board_checkers()
{
    M_run_test_suite(test_board_edge_checkers)
    M_run_test_suite(test_continuous_counter)
}
