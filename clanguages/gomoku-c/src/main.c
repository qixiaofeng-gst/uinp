#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>

#include "board.h"
#include "macro-constants.h"
#include "table-utilities.h"
#include "terminal-utilities.h"

const wchar_t first_hand = m_initializer_first_hand;
const wchar_t second_hand = L'X';
const int first_piece = 0B0001;
const int second_piece = 0B1000;

wchar_t
_pass_hand()
{
    static wchar_t currentHand = m_initializer_first_hand;

    wchar_t returnHand = currentHand;
    if (first_hand == currentHand) {
        currentHand = second_hand;
    } else {
        currentHand = first_hand;
    }
    return returnHand;
}

inline
void
_locate_cursor(int const upOffset, int const leftOffset)
{
    wprintf(L"\033[%dA\033[%dD", upOffset * 2 + 1, leftOffset * 2 + 2);
}

inline
void
_reset_cursor_location()
{
    wprintf(L"\033[u\033[s");
}

inline
void
_clear_message()
{
    wprintf(L"\033[K");
}

typedef struct {
    int const x;
    int const y;
    int const pieceFlag;
} HandDescription;

HandDescription
_ai_play_hand(HandDescription const referenceHand)
{
    return referenceHand;
}

int
main(int argc, char const *argv[])
{
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

    while(false == (exit_flag == input)) {
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
                    put_piece_at(x, y, (first_hand == hand) ? first_piece : second_piece);
                    HandDescription handDesc = {
                        .x = 9,
                        .y = 9,
                        .pieceFlag = 9,
                    };
                    _ai_play_hand(handDesc);
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
    return systemResult;
}
