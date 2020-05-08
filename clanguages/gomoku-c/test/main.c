#include "board-test.h"

// ======= Temporary block for quick prototype.
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// TODO

void
tryRandomNumber()
{
    srand(time(NULL));
    printf("Random numner: %d\n", rand() % 100);
}
// ======= Temporary block end.

int
main()
{
    testBoardCheckers();

    // ======= Temporary block for quick prototype.
    tryRandomNumber();
    // ======= Temporary block end.

    return 0;
}
