#include <wchar.h>
#include <locale.h>
#include <stdbool.h>

int main(int argc, char const *argv[]) {
    wprintf(L"%s\n", setlocale(LC_ALL, ""));
    wprintf(L">>>>>>> Belows are sandbox output:\n");

    #define N L'\n', /* New line. */
    #define x L'\x253C', /* Cross line single-ed. */
    #define H L'\x2550', /* Horizontal line double-ed. */
    #define h L'\x2500', /* Horizontal line single-ed. */
    #define V L'\x2551', /* Vertical line double-ed. */
    #define v L'\x2502', /* Vertical line single-ed. */
    #define L L'\x2554', /* Left-top corner double-ed. */
    #define R L'\x2557', /* Right-top corner double-ed. */
    #define l L'\x255A', /* Left-bottom corner double-ed. */
    #define r L'\x255D', /* Right-bottom corner double-ed. */
    #define M L'\x255F', /* Middle-left double-ed. */
    #define m L'\x2562', /* Middle-right double-ed. */
    #define C L'\x2564', /* Center-top double-ed. */
    #define c L'\x2567', /* Center-bottom double-ed. */
    #define O L'O', /* First-hand piece. */
    #define o L'X', /* Second. */
    #define S L' ', /* Second. */
    const wchar_t table[] = {
        L H H H C H H R N
        V O o S v S S V N
        M h h h x h h m N
        V S S S v S S V N
        l H H H c H H r N
        L'\0',
    };
    #undef N
    #undef x
    #undef H
    #undef h
    #undef V
    #undef v
    #undef L
    #undef l
    #undef R
    #undef r
    #undef M
    #undef m
    #undef C
    #undef c
    #undef O
    #undef o
    #undef S

    int count = 0;
    for (int i = 0x2400; i < 0x2700; ++i) {
        ++count;
        wprintf(L"%lc, %X; ", i, i);
        if ( 0 == ( count % 20 ) ) {
            wprintf(L"\n");
        }
    }

    wprintf(table);
    system("stty raw");
    const wchar_t exit_flag = L'x';
    wchar_t input = 0;
    while(false == (exit_flag == input)) {
        input = getwchar();
        wprintf(L"====>>> %X\n", input);
    }
    system("stty cooked");
    return 0;
}
