#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>

#include "board.h"
#include "macro-constants.h"
#include "table-utilities.h"
#include "terminal-utilities.h"

typedef struct {
    int const x;
    int const y;
    int const pieceFlag;
} HandDescription;

typedef void (*cb_ptr_player_t)(HandDescription const *const, HandDescription *const);

/**
TODO GameManager:
gm_start_game();
while (true) {
    gm_cb_ptr_play();
    gm_output_board();
    if (gm_is_game_end()) {
        break;
    }
}
gm_end_game();

void
gm_cb_ptr_play()
{
    nextHand = (*current_player)(prevHand);
    if (first_player == current_player) {
        current_player = second_player;
    } else {
        current_player = first_player;
    }
}
*/

wchar_t const G_first_hand = m_initializer_first_hand;
wchar_t const G_second_hand = L'X';
int const G_first_piece = 0B0001;
int const G_second_piece = 0B1000;

wchar_t
_pass_hand() {
    static wchar_t currentHand = m_initializer_first_hand;

    wchar_t returnHand = currentHand;
    if (G_first_hand == currentHand) {
        currentHand = G_second_hand;
    } else {
        currentHand = G_first_hand;
    }
    return returnHand;
}

inline
void
_locate_cursor(int const upOffset, int const leftOffset) {
    wprintf(L"\033[%dA\033[%dD", upOffset * 2 + 1, leftOffset * 2 + 2);
}

inline
void
_reset_cursor_location() {
    wprintf(L"\033[u\033[s");
}

inline
void
_clear_message() {
    wprintf(L"\033[K");
}

void
_ai_play_hand(HandDescription const *const prevHand, HandDescription *const currHand) {
    wprintf(L"This is a placeholder line for ai player.\n");
}

bool
gm_is_game_end()
{
    /**
     * TODO
     * 1.1. If there is a five go to step 2,
     * 1.2. else return false.
     * 2. Ask user to restart or exit.
     * 3.1 If user chose restart then clear board, start game and return false,
     * 3.2 else return true.
     */
    return false;
}

cb_ptr_player_t gm_cb_ptr_play = _ai_play_hand;
cb_ptr_player_t gm_cb_ptr_first_player = _ai_play_hand;
cb_ptr_player_t gm_cb_ptr_second_player = _ai_play_hand;

void
gm_switch_player()
{
    gm_cb_ptr_play = (gm_cb_ptr_play == gm_cb_ptr_first_player)
                     ? gm_cb_ptr_second_player
                     : gm_cb_ptr_first_player;
}

void
gm_output_board(HandDescription const * const currHand)
{
    /**
     * TODO
     * Just output the given hand.
     */
}

int
main(int argc, char const *argv[]) {
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
    wprintf(L"\033[s");

    int systemResult = system("stty raw");
    const int offsetLimit = m_table_logic_size - 1;
    const wchar_t exit_flag = L'x';
    wchar_t input = 0;
    int upOffset = 7;
    int leftOffset = 7;

    _locate_cursor(upOffset, leftOffset);

    /**
     * XXX Workflow:
     * 1. Human player play, blocking, return while 'm' is pressed.
     *    Press 'x' will abort the game.
     * 2. AI player play, show 'AI playing...' prompt, return while finish.
     * 3.1. If game end go to step 4,
     * 3.2. else go to step 1.
     * 4. Game end.
     *
     * HandDescription prevHand, currHand;
     * while (false == gm_is_game_end()) {
     *     gm_cb_ptr_play(&prevHand, &currHand);
     *     gm_output_board(&handDesc);
     *     gm_switch_player();
     * }
     */
    while (false == (exit_flag == input)) {
        input = getwchar();
        switch (input) {
            case L'i':
                if (upOffset < offsetLimit) {
                    wprintf(L"\033[2A");
                    upOffset++;
                }
                break;
            case L'k':
                if (upOffset > 0) {
                    wprintf(L"\033[2B");
                    upOffset--;
                }
                break;
            case L'j':
                if (leftOffset < offsetLimit) {
                    wprintf(L"\033[2D");
                    leftOffset++;
                }
                break;
            case L'l':
                if (leftOffset > 0) {
                    wprintf(L"\033[2C");
                    leftOffset--;
                }
                break;
            case L'm': {
                int x = offsetLimit - leftOffset;
                int y = offsetLimit - upOffset;
                if (is_empty_slot(x, y)) {
                    wchar_t hand = _pass_hand();
                    wprintf(L"%lc\033[D", hand);
                    put_piece_at(x, y, (G_first_hand == hand) ? G_first_piece : G_second_piece);
                    HandDescription handDesc = {
                            .x = 9,
                            .y = 9,
                            .pieceFlag = 9,
                    };
                    _ai_play_hand(&handDesc, &handDesc);
                    _reset_cursor_location();
                    _clear_message();
                    wprintf(L"HandDesc.x: %d, HandDesc.y: %d", handDesc.x, handDesc.y);
                    _reset_cursor_location();
                    _locate_cursor(upOffset, leftOffset);
                }
            }
                break;
            case L'M':
                _reset_cursor_location();
                _clear_message();
                wprintf(L"Broke input.");
                _reset_cursor_location();
                _locate_cursor(upOffset, leftOffset);
                break;
            default:
                break;
        }
    }
    wprintf(L"\033[u\033[K\n");
    systemResult = system("stty cooked");
    systemResult = system("clear");

    touch_terminal(true);
    // XXX Use exit(systemResult) can abort the program.
    return systemResult;
}
