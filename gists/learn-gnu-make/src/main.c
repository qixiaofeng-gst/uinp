#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>

#include "table-utilities.h"

#define M_initializer_first_hand L'O'

const wchar_t first_hand = M_initializer_first_hand;
const wchar_t second_hand = L'X';

int board[M_table_logic_size][M_table_logic_size];

void
clearBoard()
{
    memset(board, 0, sizeof(int) * M_table_logic_size * M_table_logic_size);
}

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

void
touchTerminal(bool isRestore)
{
    static struct termios backup;
    if (isRestore) {
        // ==================
    } else {

    }
}

int
main(int argc, char const *argv[])
{
    clearBoard();

    struct termios termInfo, termInfoBak;
    tcgetattr(0, &termInfoBak);
    tcgetattr(0, &termInfo);
    termInfo.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSAFLUSH, &termInfo);

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
    int upOffset = 0;
    int leftOffset = 0;

    wprintf(L"\033[1A");
    wprintf(L"\033[2D");
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
        case L'm':
            wprintf(L"%lc\033[D", passHand());
            break;
        default:
            break;
        }
    }
    wprintf(L"\033[999E\n");
    system("stty cooked");
    tcsetattr(0, TCSAFLUSH, &termInfoBak);
    return 0;
}
