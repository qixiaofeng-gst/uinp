#ifndef BOARD_TEST_H
#define BOARD_TEST_H
#include <stdbool.h>

bool _checkUpBorder(int coordinate);

bool _checkBottomBorder(int coordinate);

int _countContinuousSameFlag(int pieceFlag, int x, int y, int incrementX, int incrementY);

void putPieceAt(int x, int y, int pieceFlag);

void clearBoard();

#endif // guard end for BOARD_TEST_H
