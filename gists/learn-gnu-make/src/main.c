#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>

#include "board.h"
#include "macro-constants.h"
#include "table-utilities.h"
#include "terminal-utilities.h"

const wchar_t first_hand = M_initializer_first_hand;
const wchar_t second_hand = L'X';
const int first_piece = 0B0001;
const int second_piece = 0B1000;

wchar_t
passHand()
{
    static wchar_t currentHand = M_initializer_first_hand;

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
_locateCursor(int const upOffset, int const leftOffset)
{
    wprintf(L"\033[%dA\033[%dD", upOffset * 2 + 1, leftOffset * 2 + 2);
}

inline
void
_resetCursorLocation()
{
    wprintf(L"\033[u\033[s");
}

void
_printMessage(char * const message)
{
    _resetCursorLocation();
    wprintf(L"\033[K %s", message);
}

typedef struct {
    int const x;
    int const y;
    int const pieceFlag;
} HandDescription;

HandDescription
ai_playHand(int const referenceX, int const referenceY)
{
    HandDescription hand = {
        .x = 9,
        .y = 9,
        .pieceFlag = 9,
    };
    return hand;
}

int
main(int argc, char const *argv[])
{
    clearBoard();
    touchTerminal(false);
    turnOffEcho();

    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    HandDescription dummyHand = ai_playHand(0, 0);
    wprintf(L"dummyHand.pieceFlag: %d\n", dummyHand.pieceFlag);
    wprintf(L">>>>>>> Belows are sandbox output:\n");
    generateTableFile();
    wchar_t tableInMemory[M_table_string_length];
    loadTableFromFile(tableInMemory);
    wprintf(L"Length to read: %d, logic size: %d\n", M_table_string_length, M_table_logic_size);
    wprintf(L"%ls", tableInMemory);
    wprintf(L"\033[s");

    int systemResult = system("stty raw");
    const int offsetLimit = M_table_logic_size - 1;
    const wchar_t exit_flag = L'x';
    wchar_t input = 0;
    int upOffset = 7;
    int leftOffset = 7;

    _locateCursor(upOffset, leftOffset);

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
                if (isEmptySlot(x, y)) {
                    wchar_t hand = passHand();
                    wprintf(L"%lc\033[D", hand);
                    putPieceAt(x, y, (first_hand == hand) ? first_piece : second_piece);
                }
            }
            break;
        case L'M':
            _printMessage("Broke input.");
            _resetCursorLocation();
            _locateCursor(upOffset, leftOffset);
            break;
        default:
            break;
        }
    }
    wprintf(L"\033[u\033[K\n");
    systemResult = system("stty cooked");
    systemResult = system("clear");

    touchTerminal(true);
    return systemResult;
}
