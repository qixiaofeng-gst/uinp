#include "testool.h"
#include "board-test.h"
#include "macro-constants.h"

void
testBoardEdgeCheckers()
{
    M_test_int(_checkBottomBorder(14), true)
    M_test_int(_checkBottomBorder(13), true)
    M_test_int(_checkBottomBorder(15), false)

    M_test_int(_checkUpBorder(-1), false)
    M_test_int(_checkUpBorder(0), true)
    M_test_int(_checkUpBorder(1), true)
}

void
testContinuousCounter()
{
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
}

void
testBoardCheckers()
{
    M_run_test_suite(testBoardEdgeCheckers)
    M_run_test_suite(testContinuousCounter)
}
