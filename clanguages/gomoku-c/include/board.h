#ifndef BOARD_H
#define BOARD_H

void clear_board();
bool is_empty_slot(int x, int y);
bool is_game_end(int x, int y, int pieceFlag);
void put_piece_at(int x, int y, int pieceFlag);
bool is_same_flag(int pieceFlag, int x, int y);

#endif // guard end for BOARD_H
