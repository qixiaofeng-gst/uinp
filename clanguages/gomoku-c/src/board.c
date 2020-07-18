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
    count_continuous_same_flag(pieceFlag, x, y, iX1, iY1) \
    + count_continuous_same_flag(pieceFlag, x, y, iX2, iY2)) \
) {\
    return true; \
}

typedef bool (*cb_ptr_checker_t)(int);

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
check_up_border(int coordinate)
{
    return coordinate > -1;
}

bool
check_bottom_border(int coordinate)
{
    return coordinate < m_table_logic_size;
}

cb_ptr_checker_t
get_checker(bool isIncreasing)
{
    return isIncreasing ? check_bottom_border : check_up_border;
}

int
count_continuous_same_flag(int pieceFlag, int x, int y, int incrementX, int incrementY)
{
    int count = 0;
    if (0 == incrementX) {
        cb_ptr_checker_t checker = get_checker(incrementY > 0);
        for (int i = y + incrementY; checker(i); i += incrementY) {
            M_count_piece(board[x][i])
        }
        return count;
    } else if (0 == incrementY) {
        cb_ptr_checker_t checker = get_checker(incrementX > 0);
        for (int i = x + incrementX; checker(i); i += incrementX) {
            M_count_piece(board[i][y])
        }
        return count;
    }
    cb_ptr_checker_t xChecker = get_checker(incrementX > 0);
    cb_ptr_checker_t yChecker = get_checker(incrementY > 0);
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
