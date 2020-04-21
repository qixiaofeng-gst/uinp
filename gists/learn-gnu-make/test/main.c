#include <stdio.h>

#include "board-test.h"
#include "macro-constants.h"

char * const trueFlag = "\033[32mtrue\033[0m";
char * const falseFlag = "\033[31mfalse\033[0m";

#define TEST_INT(value, expected) printf( \
    "Passed: [%s], testing value[%s], value: %d, expected: %d\n", \
    (value == expected) ? trueFlag : falseFlag, \
    #value, \
    value, \
    expected \
)

int
main()
{
    int count = 0;
    for ( ; _checkBottomBorder(count); ++count) {}
    TEST_INT(count, M_table_logic_size);

    int start = 4;
    for ( ; _checkUpBorder(start); --start) {}
    TEST_INT(start, 0);
    return 0;
}
