#ifndef BOARD_TEST_H
#define BOARD_TEST_H

#include <stdbool.h>

#include "types.h"

// Belows for ../src/board.c.

bool p_check_up_border(int coordinate);

bool p_check_bottom_border(int coordinate);

int p_count_continuous_same_flag(Board const *board, int pieceFlag, int x, int y, int incrementX, int incrementY);

// Belows for board-test.c.
void test_board_checkers();

#endif // guard end for BOARD_TEST_H
