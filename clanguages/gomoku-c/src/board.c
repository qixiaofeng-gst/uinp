#include <string.h>
#include <stdbool.h>

#include "board.h"

typedef bool (*cb_checker_t)(int);

int const win_count = 4;

void
clear_board(Board *board) {
    memset(board, m_to_memset, sizeof(Board)); //XXX Leave it here for memo. Useless
    for (int i = 0; i < m_table_logic_size; ++i) {
        for (int j = 0; j < m_table_logic_size; ++j) {
            board->grids[i][j] = m_empty_appeance;
        }
    }
}

bool
is_empty_slot(Board const *board, Point const *point) {
    return m_empty_appeance == board->grids[point->x][point->y];
}

bool
is_on_board(Board const *board, HandDescription const *hand) {
    return hand->appearance == board->grids[hand->x][hand->y];
}

bool
p_check_up_border(int coordinate) {
    return coordinate > -1;
}

bool
p_check_bottom_border(int coordinate) {
    return coordinate < m_table_logic_size;
}

cb_checker_t
p_get_checker(bool isIncreasing) {
    return isIncreasing ? p_check_bottom_border : p_check_up_border;
}

int
p_count_continuous_same_flag(Board const *board, int pieceFlag, int x, int y, int incrementX, int incrementY) {
    #define M_count_piece(target) \
    if (pieceFlag == target) { \
        ++count; \
    } else { \
        return count; \
    }

    int count = 0;
    if (0 == incrementX) {
        cb_checker_t checker = p_get_checker(incrementY > 0);
        for (int i = y + incrementY; checker(i); i += incrementY) {
            M_count_piece(board->grids[x][i])
        }
        return count;
    } else if (0 == incrementY) {
        cb_checker_t checker = p_get_checker(incrementX > 0);
        for (int i = x + incrementX; checker(i); i += incrementX) {
            M_count_piece(board->grids[i][y])
        }
        return count;
    }
    cb_checker_t xChecker = p_get_checker(incrementX > 0);
    cb_checker_t yChecker = p_get_checker(incrementY > 0);
    for (int i = x + incrementX, j = y + incrementY; xChecker(i) && yChecker(j); i += incrementX, j += incrementY) {
        M_count_piece(board->grids[i][j])
    }
    return count;

    #undef M_count_piece
}

bool
is_game_end(Board const *board, HandDescription const *hand) {
    #define M_check_game_end(iX1, iY1, iX2, iY2) if (win_count <= ( \
        p_count_continuous_same_flag(board, hand->appearance, hand->x, hand->y, iX1, iY1) \
        + p_count_continuous_same_flag(board, hand->appearance, hand->x, hand->y, iX2, iY2)) \
    ) {\
        return true; \
    }

    // Check horizontal.
    M_check_game_end(1, 0, -1, 0)
    // Check vertical.
    M_check_game_end(0, 1, 0, -1)
    // Check top-left to bottom-right.
    M_check_game_end(1, 1, -1, -1)
    // Check top-right to bottom-left.
    M_check_game_end(-1, 1, 1, -1)

    return false;

    #undef M_check_game_end
}

void
put_piece_at(Board *board, HandDescription const *hand) {
    board->grids[hand->x][hand->y] = hand->appearance;
}
