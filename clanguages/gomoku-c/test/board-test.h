#ifndef BOARD_TEST_H
#define BOARD_TEST_H
#include <stdbool.h>

// Belows for ../src/board.c.

bool check_up_border(int coordinate);

bool check_bottom_border(int coordinate);

int count_continuous_same_flag(int pieceFlag, int x, int y, int incrementX, int incrementY);

void put_piece_at(int x, int y, int pieceFlag);

void clear_board();

bool is_game_end(int x, int y, int pieceFlag);

// Belows for board-test.c.
void test_board_checkers();

#endif // guard end for BOARD_TEST_H
