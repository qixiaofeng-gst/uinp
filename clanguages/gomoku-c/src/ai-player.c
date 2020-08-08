//
// Created by qixiaofeng on 2020/7/30.
//

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "board.h"
#include "ai-player.h"
#include "macro-functions.h"

#define M_debug_ppp(point) p_print_point(point, __FUNCTION__, __LINE__);

bool p_enable_print = true;
wchar_t p_g_appearance = m_empty_appeance;
unsigned values[m_table_logic_size][m_table_logic_size];

/*!
 * 1. The dot (.) means current point.
 * 2. The star (*) means any point.
 * 3. The character (o) means ally point.
 * 4. The character (x) means non-ally point.
 * 5. The underscore (_) means empty point.
 * 6. The vertical line (|) means enemy or border point.
 * 7. The colon (:) means ally or empty point.
 */
char const *const patterns[] = {
        // dead area, 0 score
        "\x02|.***|\xFF", "\x03|*.**|\xFF", "\x03|*.*|\xFF", "\x02|.**|\xFF", "\x02|.*|\xFF", "\x02|.|\xFF",
        // low area, 1 score
        "\x02|.::::|\x01", "\x03|:.:::|\x01", "\x04|::.::|\x01",
        // one dead end empty area
        "\x02|.____\x02", "\x03|_.____\x02", "\x04|__.____\x02", "\x05|___.____\x02",
        // dead 1
        "\x03|o.____\x03", "\x04|o_.___\x03", "\x05|o__.__\x03",
        // open empty area
        "\x05____.____\x04",
        // alive 1, constructing alive 2
        "\x03__.o__\x05", "\x02_._o__\x05", "\x02_.__o__\x05",
        // dead 2, constructing half 3
        "\x04|oo.__x\x06", "\x05|oo_._x\x06", "\x06|oo__.x\x06",
        // alive 2, constructing alive 3
        "\x03x_.oo_x\x07", "\x04x_o.o_x\x07", "\x04x_o._o_x\x07",
        // half 3, constructing winning 4
        "\x02x._ooo|\x08", "\x03x_.ooo|\x08", "\x04x_o.oo|\x08", "\x05x_oo.o|\x08", "\x06x_ooo.|\x08",
        // alive 3, constructing winning 4
        "\x03x_.ooo_x\x09", "\x04x_o.oo_x\x09",
        // winning 4, win point
        "\x02x.oooox\x0A", "\x03xo.ooox\x0A", "\x04xoo.oox\x0A",
};
size_t const patterns_size = sizeof(patterns) / sizeof(char *);

bool
p_wild_pv(Board const *b, Point const *p, wchar_t a) {
    (void) b;
    (void) p;
    (void) a;
    return true;
}

bool
p_ally_pv(Board const *b, Point const *p, wchar_t a) {
    return a == b->grids[p->x][p->y];
}

cb_point_validator_t
p_get_point_validator(char pattern) {
    switch (pattern) {
        case '*':
            return p_wild_pv;
        case 'o':
            return p_ally_pv;
        // TODO more point validators.
        default:
            return p_wild_pv;
    }
}

int
p_validate_patterns() {
    for (size_t i = 0; i < patterns_size; ++i) {
        char const *pattern = patterns[i];
        size_t const size = strlen(pattern);
        size_t const dotIndex = pattern[0];
        if (dotIndex >= size) {
            return i;
        }
        bool const dotIsValid = (pattern[dotIndex] == '.');
        if (false == dotIsValid) {
            return i;
        }
        bool const scoreIsValid = (pattern[size - 1] < 0x0B);
        if (false == scoreIsValid) {
            return i;
        }
    }
    return -1; // means valid
}

wchar_t
p_ai_board_get(Board const *board, Point const *const point) {
    return board->grids[point->x][point->y];
}

void
p_print_point(Point const *point, char const *functionName, int lineNumber) {
    if (false == p_enable_print) {
        return;
    }
    printf(
            "Print Point(%d, %d) at %s [%d]. Address: %p.\n",
            point->x, point->y, functionName, lineNumber, point
    );
}

unsigned
p_default_evaluator(Board const *board, Point const *sourcePoint, Point const *targetPoint) {
    if (board->grids[sourcePoint->x][sourcePoint->y] == board->grids[targetPoint->x][targetPoint->y]) {
        return 1;
    }
    return 0;
}

unsigned
p_evaluate_point(Board const *board, Point const *const sourcePoint, cb_evaluator_t cbEvaluator) {
    /*!
     * Empty grid is with initial value 1.
     * Ocuppied grid is with value 0.
     * Accumulate value with counting neighbors.
     *
     * Stop while meet enemy piece or border.
     */
    bool notEmpty = (false == (m_empty_appeance == board->grids[sourcePoint->x][sourcePoint->y]));
    if (notEmpty) {
        return 0;
    }
    return cbEvaluator(board, sourcePoint, sourcePoint);
}

wchar_t
ai_get_appearance() {
    return p_g_appearance;
}

void
ai_set_appearance(wchar_t const targetAppearance) {
    p_g_appearance = targetAppearance;
    M_clear_board(values, 0u)

    Point point = {
            .x = 0,
            .y = 0
    };
    M_debug_ppp(&point)
}

void
ai_play_hand(Board *board, HandDescription const *prevHand, HandDescription *currHand) {
    currHand->x = prevHand->x; //XXX Eliminate unused warning.

    /*TODO Check if there is any piece occupying the hand position. */
    currHand->x = 0;
    currHand->y = 0;
    currHand->appearance = p_g_appearance;
    put_piece_at(board, currHand);
}

#undef M_debug_ppp
