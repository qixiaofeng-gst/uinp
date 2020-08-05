#include <locale.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include "types.h"
#include "log.h"
#include "board.h"
#include "ai-player.h"
#include "macro-constants.h"
#include "table-utilities.h"
#include "terminal-utilities.h"

/*!
 * All the gm_ prefix means that it belongs to GameManager.
 * GameManager is a concept object. It does not have an real instance.
 */

int const G_offset_limit = m_table_logic_size - 1;
wchar_t const G_pass_flag = L'p';
wchar_t g_input_char = 0;
int g_up_offset = 7;
int g_left_offset = 7;

Board board;

void
exit_program() {
    wprintf(L"\033[u\033[K\n");
    int systemResult = 0;
    systemResult += system("stty cooked");
    systemResult += system("clear");

    touch_terminal(true);
    fclose(get_debug_log_file());
    exit(systemResult);
}

void
locate_cursor(int const upOffset, int const leftOffset) {
    wprintf(L"\033[%dA\033[%dD", upOffset * 2 + 1, leftOffset * 2 + 2);
}

void
reset_cursor_location() {
    wprintf(L"\033[u\033[s");
}

void
clear_message() {
    wprintf(L"\033[K");
}

void
print_message(wchar_t const *msgFormat, ...) {
    va_list args;
    va_start(args, msgFormat);

    reset_cursor_location();
    clear_message();
    vwprintf(msgFormat, args);
    reset_cursor_location();
    locate_cursor(g_up_offset, g_left_offset);

    va_end(args);
}

void
human_play_hand(HandDescription const *const prevHand, HandDescription *const currHand) {
    print_message(L"Your turn. PrevHand: %d, %d.", prevHand->x, prevHand->y);
    while (false == (G_pass_flag == g_input_char)) {
        g_input_char = getwchar();
        switch (g_input_char) {
            case L'x':
                exit_program();
                return;
            case L'i':
                if (g_up_offset < G_offset_limit) {
                    wprintf(L"\033[2A");
                    g_up_offset++;
                }
                break;
            case L'k':
                if (g_up_offset > 0) {
                    wprintf(L"\033[2B");
                    g_up_offset--;
                }
                break;
            case L'j':
                if (g_left_offset < G_offset_limit) {
                    wprintf(L"\033[2D");
                    g_left_offset++;
                }
                break;
            case L'l':
                if (g_left_offset > 0) {
                    wprintf(L"\033[2C");
                    g_left_offset--;
                }
                break;
            case L'm': {
                int x = G_offset_limit - g_left_offset;
                int y = G_offset_limit - g_up_offset;
                Point point = {.x = x, .y = y};
                if (is_empty_slot(&board, &point)) {
                    currHand->x = x;
                    currHand->y = y;
                    currHand->appearance = m_first_appearance;
                    put_piece_at(&board, currHand);
                    return;
                }
            }
                break;
            case L'M':
                print_message(L"Cleared message.");
                break;
            default:
                break;
        }
    }
    g_input_char = 0;
}

bool
gm_is_game_end(HandDescription const *const prevHand) {
    if ((m_invalid_coord == prevHand->x) || (m_invalid_coord == prevHand->y)) {
        return false;
    }
    if (is_game_end(&board, prevHand)) {
        // TODO Ask user to restart or exit.
        return true;
    }
    return false;
}

cb_player_t gm_cb_first_player = human_play_hand;
cb_player_t gm_cb_second_player = ai_play_hand;
cb_player_t gm_cb_play = human_play_hand;

void
gm_switch_player() {
    gm_cb_play = (gm_cb_play == gm_cb_first_player)
                 ? gm_cb_second_player
                 : gm_cb_first_player;
}

void
gm_output_board(HandDescription const *const currHand) {
    reset_cursor_location();
    locate_cursor(G_offset_limit - currHand->y, G_offset_limit - currHand->x);
    wprintf(L"%lc\033[D", currHand->appearance);
}

int
main() {
    debug_print("Debug print test. %p", &G_offset_limit);
    clear_board(&board);
    touch_terminal(false);
    turn_off_echo();

    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    wprintf(L">>>>>>> Belows are sandbox output:\n");
    generate_table_file();
    wchar_t tableInMemory[m_table_string_length];
    load_table_from_file(tableInMemory);
    wprintf(L"Length to read: %d, logic size: %d\n", m_table_string_length, m_table_logic_size);
    wprintf(L"%ls", tableInMemory);

    //! Below line saved cursor position.
    wprintf(L"\033[s");

    int systemResult = system("stty raw");

    locate_cursor(g_up_offset, g_left_offset);

    HandDescription prevHand = {
            .x = m_invalid_coord,
            .y = m_invalid_coord,
            .appearance = m_empty_appeance,
    }, currHand = {
            .x = m_invalid_coord,
            .y = m_invalid_coord,
            .appearance = m_first_appearance,
    };
    ai_set_appearance(m_second_appearance);
    while (false == gm_is_game_end(&prevHand)) {
        gm_cb_play(&prevHand, &currHand);
        gm_output_board(&currHand);
        gm_switch_player();

        prevHand.x = currHand.x;
        prevHand.y = currHand.y;
        prevHand.appearance = currHand.appearance;
    }

    exit_program();
    return systemResult;
}
