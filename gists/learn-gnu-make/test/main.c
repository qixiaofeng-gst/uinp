#include <stdio.h>

#include "board-test.h"
#include "macro-constants.h"

char * const trueFlag = "\033[32mpassed\033[0m";
char * const falseFlag = "\033[31mfailed\033[0m";

int totalTestCount = 0;
int passedTestCount = 0;
int failedTestCount = 0;
#define M_test_int(value, expected) reportIntegerTest( \
    #value, \
    value, \
    expected \
);

void
reportIntegerTest(char * const testedName, int const value, int const expected)
{
    bool isPassed = (value == expected);
    printf(
        "Test [%s], testing value[%s], value: %d, expected: %d\n",
        isPassed ? trueFlag : falseFlag,
        testedName,
        value,
        expected
    );
    ++totalTestCount;
    if (isPassed) {
        ++passedTestCount;
    } else {
        ++failedTestCount;
    }
}

void
resetTestCounts()
{
    totalTestCount = 0;
    passedTestCount = 0;
    failedTestCount = 0;
}

void
reportTestSummary()
{
    printf(
        "Total count: %d; passed test count: \033[32m%d\033[0m; \
failed test count: \033[31m%d\033[0m.\n",
        totalTestCount,
        passedTestCount,
        failedTestCount
    );
}

int
main()
{
    resetTestCounts();
    M_test_int(_checkBottomBorder(14), true)
    M_test_int(_checkBottomBorder(13), true)
    M_test_int(_checkBottomBorder(15), false)

    M_test_int(_checkUpBorder(-1), false)
    M_test_int(_checkUpBorder(0), true)
    M_test_int(_checkUpBorder(1), true)

    int const flagOne = 9;
    int const flagTwo = 3;
    clearBoard();
    for (int i = 0; i < 4; ++i) {
        putPieceAt(0, i, flagOne);
    }
    M_test_int(isGameEnd(0, 3, flagOne), false)
    M_test_int(_countContinuousSameFlag(flagOne, 0, 3, 0, -1), 3)
    M_test_int(_countContinuousSameFlag(flagOne, 0, 1, 0, -1), 1)
    M_test_int(_countContinuousSameFlag(flagOne, 0, 1, 0, 1), 2)
    M_test_int(_countContinuousSameFlag(flagOne, 0, 0, 0, 1), 3)
    putPieceAt(0, 1, flagTwo);
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, 0, 1), 0)
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, 0, -1), 0)
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, 1, -1), 0)
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, 1, 1), 0)
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, 1, 0), 0)
    M_test_int(_countContinuousSameFlag(flagTwo, 0, 1, -1, 0), 0)
    M_test_int(_countContinuousSameFlag(M_empty_slot, 0, 5, 0, -1), 1)
    M_test_int(_countContinuousSameFlag(M_empty_slot, 0, 5, 0, 1), 9)

    clearBoard();
    for (int i = 0; i < 9; ++i) {
        for (int j = 0; j < 9; ++j) {
            putPieceAt(i, j, flagOne);
        }
    }
    putPieceAt(9, 9, flagTwo);
    M_test_int(isGameEnd(3, 3, flagOne), true)
    M_test_int(_countContinuousSameFlag(flagOne, 1, 1, -1, -1), 1)
    M_test_int(_countContinuousSameFlag(flagOne, 1, 1, 1, 1), 7)
    reportTestSummary();
    return 0;
}
