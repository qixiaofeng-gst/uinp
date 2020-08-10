#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "board.h"
#include "macro-functions.h"

typedef bool (*cb_checker_t)(int);

int const win_count = 4;

void
clear_board(Board *board) {
    memset(board, m_to_memset, sizeof(Board)); //XXX Leave it here for memo. Useless
    M_clear_board(board->grids, m_empty_appearance)
}

void
print_region_edge(char *edge, size_t width) {
    for (size_t i = 0; i < width; ++i) {
        edge[i] = '-';
    }
    printf("%s\n", edge);
}

void print_board_region(Board const *board, BoardRegion const *region) {
    (void) board;
    char line[region->w + 1];
    line[region->w] = '\0';
    print_region_edge(line, region->w);
    for (size_t i = 0; i < region->h; ++i) {
        for (size_t j = 0; j < region->w; ++j) {
            wchar_t b = board->grids[j][i];
            switch (b) {
                case m_first_appearance:
                    line[j] = 'X';
                    break;
                case m_second_appearance:
                    line[j] = 'O';
                    break;
                case m_empty_appearance:
                default:
                    line[j] = ' ';
            }
        }
        printf("%s\n", line);
    }
    print_region_edge(line, region->w);
}

bool
is_empty_slot(Board const *board, Point const *point) {
    return m_empty_appearance == board->grids[point->x][point->y];
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

bool
p_validate_coordinate(int coordinate) {
    return (
            p_check_up_border(coordinate) &&
            p_check_bottom_border(coordinate)
    );
}

bool
validate_board_point(Point const *point) {
    return (
            p_validate_coordinate(point->x) &&
            p_validate_coordinate(point->y)
    );
}

cb_checker_t
p_get_checker(bool isIncreasing) {
    return isIncreasing ? p_check_bottom_border : p_check_up_border;
}

int
p_count_continuous_same_flag(Board const *board, int pieceFlag, int x, int y, int incrementX, int incrementY) {
    int count = 0;
    cb_checker_t xChecker = p_get_checker(incrementX > 0);
    cb_checker_t yChecker = p_get_checker(incrementY > 0);
    for (int i = x + incrementX, j = y + incrementY; xChecker(i) && yChecker(j); i += incrementX, j += incrementY) {
        if (pieceFlag == board->grids[i][j]) {
            ++count;
        } else {
            return count;
        }
    }
    return count;
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
