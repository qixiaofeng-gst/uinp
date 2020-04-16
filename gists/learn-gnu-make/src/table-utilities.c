#include <wchar.h>
#include <stdio.h>
#include <stdbool.h>

#include "table-utilities.h"

const char *tableFileName = "table.dat";
const int table_string_length = 992;

void
loadTableFromFile(wchar_t *tableInMemory)
{
    FILE *tableFile = fopen(tableFileName, "r");
    if (NULL == tableFile) {
        return;
    }
    fread(tableInMemory, sizeof(wchar_t), table_string_length, tableFile);
    fclose(tableFile);
}

void
showUnicodeTable()
{
    int count = 0;
    for (int i = 0x2400; i < 0x2700; ++i) {
        ++count;
        wprintf(L"%lc, %X; ", i, i);
        if ( 0 == ( count % 20 ) ) {
            wprintf(L"\n");
        }
    }
}

bool
isTableFileExist()
{
    FILE *tableFile = fopen(tableFileName, "r");
    if (NULL == tableFile) {
        return false;
    }
    fclose(tableFile);
    return true;
}

void
generateTableFile()
{
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
    #define S L' ', /* Second. */
    const wchar_t table[] = {
        L H C H C H C H C H C H C H C H C H C H C H C H C H C H C H R N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        M h x h x h x h x h x h x h x h x h x h x h x h x h x h x h m N
        V S v S v S v S v S v S v S v S v S v S v S v S v S v S v S V N
        l H c H c H c H c H c H c H c H c H c H c H c H c H c H c H r L'\0'
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
    if (isTableFileExist()) {
        return;
    }

    FILE *outputFile = fopen(tableFileName, "w");
    fwrite(table, sizeof(wchar_t), wcslen(table) + 1, outputFile);
    fclose(outputFile);
}
