#include <locale.h>
#include <stdio.h>
#include <wchar.h>
#include <stdbool.h>
#include <stdlib.h>

#include "table-utilities.h"

int
main(int argc, char const *argv[])
{
    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    wprintf(L">>>>>>> Belows are sandbox output:\n");
    generateTableFile();

    system("stty raw");
    const wchar_t exit_flag = L'x';
    wchar_t input = 0;
    while(false == (exit_flag == input)) {
        input = getwchar();
        wprintf(L"====>>> %X\n\r", input);
    }
    system("stty cooked");
    return 0;
}
