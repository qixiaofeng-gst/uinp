//
// Created by qixiaofeng on 2020/7/30.
//

#ifndef GOMOKU_C_AI_PLAYER_H
#define GOMOKU_C_AI_PLAYER_H

#include "types.h"

wchar_t ai_get_appearance();

void ai_set_appearance(wchar_t targetAppearance);

void ai_play_hand(HandDescription const *prevHand, HandDescription *currHand);

#endif //GOMOKU_C_AI_PLAYER_H
