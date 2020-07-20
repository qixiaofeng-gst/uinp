#include <locale.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include "board.h"
#include "macro-constants.h"
#include "table-utilities.h"
#include "terminal-utilities.h"

/*!
 * All the gm_ prefix means that it belongs to GameManager.
 * GameManager is a concept object. It does not have an real instance.
 */

FILE *
get_debug_log() {
    static FILE *debug_log = NULL;
    if (NULL == debug_log) {
        debug_log = fopen("debug.log", "w");
    }
    return debug_log;
}

void
debug_print(char const *msgFormat, ...) {
    va_list args;
    va_start(args, msgFormat);
    vfprintf(get_debug_log(), msgFormat, args);
    va_end(args);
}

typedef struct {
    int x;
    int y;
    wchar_t pieceAppearance;
} HandDescription;

typedef void (*cb_ptr_player_t)(HandDescription const *const, HandDescription *const);

wchar_t const G_first_hand = m_initializer_first_hand;
wchar_t const G_second_hand = L'X';
int const G_first_piece = 0B0001;
int const G_second_piece = 0B1000;

int const G_offset_limit = m_table_logic_size - 1;
int const G_invalid_coord = -1;
wchar_t const G_pass_flag = L'p';
wchar_t g_input_char = 0;
int g_up_offset = 7;
int g_left_offset = 7;

void
exit_program() {
    wprintf(L"\033[u\033[K\n");
    int systemResult = 0;
    systemResult += system("stty cooked");
    systemResult += system("clear");

    touch_terminal(true);
    fclose(get_debug_log());
    exit(systemResult);
}

wchar_t
switch_piece_appearance() {
    static wchar_t currentHand = m_initializer_first_hand;

    wchar_t returnHand = currentHand;
    if (G_first_hand == currentHand) {
        currentHand = G_second_hand;
    } else {
        currentHand = G_first_hand;
    }
    return returnHand;
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
ai_play_hand(HandDescription const *const prevHand, HandDescription *const currHand) {
    print_message(L"AI's turn. PrevHand: %d, %d.", prevHand->x, prevHand->y);
    currHand->x = 0;
    currHand->y = 0;
    currHand->pieceAppearance = switch_piece_appearance();
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
                if (is_empty_slot(x, y)) {
                    wchar_t hand = switch_piece_appearance();

                    currHand->x = x;
                    currHand->y = y;
                    currHand->pieceAppearance = hand;

                    put_piece_at(x, y, (G_first_hand == hand) ? G_first_piece : G_second_piece);
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

int
get_piece_flag(wchar_t const pieceAppearance) {
    return (G_first_hand == pieceAppearance) ? G_first_piece : G_second_piece;
}

bool
gm_is_game_end(HandDescription const *const prevHand) {
    if ((G_invalid_coord == prevHand->x) || (G_invalid_coord == prevHand->y)) {
        return false;
    }
    // TODO There is BUG in is_game_end method. Cannot detect slope lines.
    if (is_game_end(prevHand->x, prevHand->y, get_piece_flag(prevHand->pieceAppearance))) {
        // TODO Ask user to restart or exit.
        return true;
    }
    return false;
}

cb_ptr_player_t gm_cb_ptr_first_player = human_play_hand;
cb_ptr_player_t gm_cb_ptr_second_player = ai_play_hand;
cb_ptr_player_t gm_cb_ptr_play = human_play_hand;

void
gm_switch_player() {
    gm_cb_ptr_play = (gm_cb_ptr_play == gm_cb_ptr_first_player)
                     ? gm_cb_ptr_second_player
                     : gm_cb_ptr_first_player;
}

void
gm_output_board(HandDescription const *const currHand) {
    reset_cursor_location();
    locate_cursor(G_offset_limit - currHand->y, G_offset_limit - currHand->x);
    wprintf(L"%lc\033[D", currHand->pieceAppearance);
}

int
main() {
    debug_print("Debug print test. %p", &G_first_hand);
    clear_board();
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
            .x = G_invalid_coord,
            .y = G_invalid_coord,
            .pieceAppearance = G_first_hand,
    }, currHand = {
            .x = G_invalid_coord,
            .y = G_invalid_coord,
            .pieceAppearance = G_first_hand,
    };
    while (false == gm_is_game_end(&prevHand)) {
        gm_cb_ptr_play(&prevHand, &currHand);
        gm_output_board(&currHand);
        gm_switch_player();

        prevHand.x = currHand.x;
        prevHand.y = currHand.y;
        prevHand.pieceAppearance = currHand.pieceAppearance;
    }

    exit_program();
    return systemResult;
}
