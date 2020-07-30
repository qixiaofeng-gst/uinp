//
// Created by qixiaofeng on 2020/7/30.
//

#include <wchar.h>

#include "ai_player.h"

wchar_t p_g_appearance = L'X';

void
ai_play_hand(HandDescription const *const prevHand, HandDescription *const currHand) {
    /*TODO Check if there is any piece occupying the hand position. */
    currHand->x = 0;
    currHand->y = 0;
    currHand->appearance = p_g_appearance;
}
