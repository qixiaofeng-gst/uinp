#include <string.h>
#include <stdbool.h>

#include "macro-constants.h"

int board[M_table_logic_size][M_table_logic_size];

void
clearBoard()
{
    memset(board, M_empty_slot, sizeof(int) * M_table_logic_size * M_table_logic_size);
}

bool
isEmptySlot(int x, int y)
{
    return 0 == board[x][y];
}

void
putPieceAt(int x, int y, int pieceFlag)
{
    board[x][y] = pieceFlag;

    /*
    Quadrants:
       4 | 1
    -----------
       3 | 2
    */
}
