//
// Created by qixiaofeng on 2020/7/30.
//

#include <stdio.h>
#include <stdbool.h>

#include "ai-player.h"
#include "macro-constants.h"

#define M_debug_ppp(point) p_print_point(point, __FUNCTION__, __LINE__);

typedef int (* cb_evaluator_t)(Point const * const sourcePoint, Point const * const targetPoint);

bool p_enable_print = true;
wchar_t p_g_appearance = L'X';
int ai_board[m_table_logic_size][m_table_logic_size];
int values[m_table_logic_size][m_table_logic_size];

void
p_print_point(Point const * const point, char const * functionName, int lineNumber) {
    if (false == p_enable_print) {
        return;
    }
    printf("Point(%d, %d) at %s [%d].\n", point->x, point->y, functionName, lineNumber);
}

int
p_evaluate_point(Point const * const sourcePoint, cb_evaluator_t cbEvaluator) {
    M_debug_ppp(sourcePoint)
    printf("ai_board size: %lu, values size: %lu", sizeof(ai_board), sizeof(values));
    return 0;
}

wchar_t
ai_get_appearance() {
    return p_g_appearance;
}

void
ai_set_appearance(wchar_t const targetAppearance) {
    p_g_appearance = targetAppearance;
}

void
ai_play_hand(HandDescription const *const prevHand, HandDescription *const currHand) {
    currHand->x = prevHand->x; //XXX Eliminate unused warning.

    /*TODO Check if there is any piece occupying the hand position. */
    currHand->x = 0;
    currHand->y = 0;
    currHand->appearance = p_g_appearance;
}
