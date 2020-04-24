#ifndef BOARD_TEST_H
#define BOARD_TEST_H
#include <stdbool.h>

// Belows for ../src/board.c.

bool _checkUpBorder(int coordinate);

bool _checkBottomBorder(int coordinate);

int _countContinuousSameFlag(int pieceFlag, int x, int y, int incrementX, int incrementY);

void putPieceAt(int x, int y, int pieceFlag);

void clearBoard();

bool isGameEnd(int x, int y, int pieceFlag);

// Belows for board-test.c.
void testBoardCheckers();

#endif // guard end for BOARD_TEST_H
