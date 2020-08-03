//
// Created by qixiaofeng on 2020/7/30.
//

#include <wchar.h>

#include "ai-player.h"

wchar_t p_g_appearance = L'X';

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
