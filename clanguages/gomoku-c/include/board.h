#ifndef BOARD_H
#define BOARD_H

#include "types.h"

void clear_board();
bool is_empty_slot(Point const *point);
bool is_game_end(HandDescription const *hand);
void put_piece_at(HandDescription const *hand);
bool is_on_board(HandDescription const *hand);

#endif // guard end for BOARD_H
