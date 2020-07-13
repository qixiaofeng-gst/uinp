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
    _count_continuous_same_flag(pieceFlag, x, y, iX1, iY1) \
    + _count_continuous_same_flag(pieceFlag, x, y, iX2, iY2)) \
) {\
    return true; \
}

int const win_count = 4;

int board[m_table_logic_size][m_table_logic_size];

void
clear_board()
{
    memset(board, m_empty_slot, sizeof(int) * m_table_logic_size * m_table_logic_size);
}

bool
is_empty_slot(int x, int y)
{
    return m_empty_slot == board[x][y];
}

inline
bool
_isSameFlag(int pieceFlag, int x, int y)
{
    return pieceFlag == board[x][y];
}

bool
_check_up_border(int coordinate)
{
    return coordinate > -1;
}

bool
_check_bottom_border(int coordinate)
{
    return coordinate < m_table_logic_size;
}

inline
bool (*
_getChecker(bool isIncreasing)
)(int)
{
    return isIncreasing ? _check_bottom_border : _check_up_border;
}

int
_count_continuous_same_flag(int pieceFlag, int x, int y, int incrementX, int incrementY)
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
is_game_end(int x, int y, int pieceFlag)
{
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
put_piece_at(int x, int y, int pieceFlag)
{
    board[x][y] = pieceFlag;
}
