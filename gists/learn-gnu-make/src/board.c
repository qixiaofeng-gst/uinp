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
    return M_empty_slot == board[x][y];
}

inline
bool
_isSameFlag(int pieceFlag, int x, int y)
{
    return pieceFlag == board[x][y];
}

bool
_checkUpBorder(int coordinate)
{
    return coordinate > 0;
}

bool
_checkBottomBorder(int coordinate)
{
    return coordinate < M_table_logic_size;
}

int
_countContinuousSameFlag(int pieceFlag, int x, int y, int incrementX, int incrementY)
{
    int count = 0;

    // for (int i = x; i < M_table_logic_size) {
    //
    // }
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
    // Check horizontals


    // Scan y--, for 2, 3 and -y axis
    // Scan y++, for 1, 4 and +y axis
}
