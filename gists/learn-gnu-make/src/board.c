#include <string.h>
#include <stdbool.h>

#include "macro-constants.h"

#define M_count_piece(target) \
if (pieceFlag == target) { \
    ++count; \
} else { \
    return count; \
}

#define M_check_game_end(iX1, iY1, iX2, iY2) if (win_count <= ( \
    _countContinuousSameFlag(pieceFlag, x, y, iX1, iY1) \
    + _countContinuousSameFlag(pieceFlag, x, y, iX2, iY2)) \
) {\
    return true; \
}

int const win_count = 4;

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
    return coordinate > -1;
}

bool
_checkBottomBorder(int coordinate)
{
    return coordinate < M_table_logic_size;
}

inline
bool (*
_getChecker(bool isIncreasing)
)(int)
{
    return isIncreasing ? _checkBottomBorder : _checkUpBorder;
}

int
_countContinuousSameFlag(int pieceFlag, int x, int y, int incrementX, int incrementY)
{
    int count = 0;
    if (0 == incrementX) {
        bool (*checker)(int) = _getChecker(incrementY > 0);
        for (int i = y + incrementY; checker(i); i += incrementY) {
            M_count_piece(board[x][i])
        }
        return count;
    } else if (0 == incrementY) {
        bool (*checker)(int) = _getChecker(incrementX > 0);
        for (int i = x + incrementX; checker(i); i += incrementX) {
            M_count_piece(board[i][y])
        }
        return count;
    }
    bool (*xChecker)(int) = _getChecker(incrementX > 0);
    bool (*yChecker)(int) = _getChecker(incrementY > 0);
    for (int i = x + incrementX; xChecker(i); i += incrementX) {
        for (int j = y + incrementY; yChecker(j); j += incrementY) {
            M_count_piece(board[i][j])
        }
    }
    return count;
}

bool
isGameEnd(int x, int y, int pieceFlag)
{
    /*
    Quadrants:
       4 | 1
    -----------
       3 | 2
    */
    // Check horizontal.
    M_check_game_end(1, 0, -1, 0)
    // Check vertical.
    M_check_game_end(0, 1, 0, -1)
    // Check top-left to bottom-right.
    M_check_game_end(1, 1, -1, -1)
    // Check top-right to bottom-left.
    M_check_game_end(-1, 1, 1, -1)

    return false;
}

void
putPieceAt(int x, int y, int pieceFlag)
{
    board[x][y] = pieceFlag;
}
