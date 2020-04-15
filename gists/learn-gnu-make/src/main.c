#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>

#include "table-utilities.h"

int
main(int argc, char const *argv[])
{
    struct termios termInfo, termInfoBak;
    tcgetattr(0, &termInfo);
    tcgetattr(0, &termInfoBak);
    termInfo.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSAFLUSH, &termInfo);

    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    wprintf(L">>>>>>> Belows are sandbox output:\n");
    generateTableFile();
    wchar_t tableInMemory[table_string_length];
    loadTableFromFile(tableInMemory);
    wprintf(L"Length to read: %d, logic size: %d\n", table_string_length, table_logic_size);
    wprintf(L"%ls", tableInMemory);

    system("stty raw");
    const int offsetLimit = table_logic_size - 1;
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
        //case L'':
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
        default:
            wprintf(L"%lc", input);
            break;
        }
    }
    wprintf(L"\033[999E\n");
    system("stty cooked");
    tcsetattr(0, TCSAFLUSH, &termInfoBak);
    return 0;
}
