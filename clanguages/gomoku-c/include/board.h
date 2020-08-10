#ifndef BOARD_H
#define BOARD_H

#include "types.h"

void clear_board(Board *board);

bool is_empty_slot(Board const *board, Point const *point);

bool is_game_end(Board const *board, HandDescription const *hand);

void put_piece_at(Board *board, HandDescription const *hand);

bool is_on_board(Board const *board, HandDescription const *hand);

bool validate_board_point(Point const *point);

// For debugging.
void print_board_region(Board const *board, BoardRegion const *region);

#endif // guard end for BOARD_H
