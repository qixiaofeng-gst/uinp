#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "table-utilities.h"
#include "terminal-utilities.h"

#define M_initializer_first_hand L'O'

const wchar_t first_hand = M_initializer_first_hand;
const wchar_t second_hand = L'X';
const int first_piece = 0B0001;
const int second_piece = 0B1000;

int board[M_table_logic_size][M_table_logic_size];

inline
void
clearBoard()
{
    memset(board, 0, sizeof(int) * M_table_logic_size * M_table_logic_size);
}
void clearBoard();

inline
bool
isEmptySlot(int x, int y)
{
    return 0 == board[x][y];
}
bool isEmptySlot();

wchar_t
passHand()
{
    static wchar_t currentHand = M_initializer_first_hand;
    #undef M_initializer_first_hand

    wchar_t returnHand = currentHand;
    if (first_hand == currentHand) {
        currentHand = second_hand;
    } else {
        currentHand = first_hand;
    }
    return returnHand;
}

int
main(int argc, char const *argv[])
{
    clearBoard();
    touchTerminal(false);
    turnOffEcho();

    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    wprintf(L">>>>>>> Belows are sandbox output:\n");
    generateTableFile();
    wchar_t tableInMemory[table_string_length];
    loadTableFromFile(tableInMemory);
    wprintf(L"Length to read: %d, logic size: %d\n", table_string_length, M_table_logic_size);
    wprintf(L"%ls", tableInMemory);

    system("stty raw");
    const int offsetLimit = M_table_logic_size - 1;
    const wchar_t exit_flag = L'x';
    wchar_t input = 0;
    int upOffset = 7;
    int leftOffset = 7;

    wprintf(L"\033[15A");
    wprintf(L"\033[16D");

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
                    if (first_hand == hand) {
                        board[x][y] = first_piece;
                    } else {
                        board[x][y] = second_piece;
                    }
                }
            }
            break;
        default:
            break;
        }
    }
    wprintf(L"\033[999E\n");
    system("stty cooked");

    touchTerminal(true);
    return 0;
}
