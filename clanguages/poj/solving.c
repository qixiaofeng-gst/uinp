/* 1059 Chutes and Ladders
Basic rules:
1. Has: player_counter += dice_number.
2. Jump to the end if stop at start.
3. Throw-again grid and stop-one-turn grid. Call them special grid.
4. Overflow ignorance.
5. First reach end win.
Input rules:
1. The first line is less than 1000 dice_numbers, and 0 < dice_number < 7.
2. The follow line is number of players.
3. Chutes and ladders description end with a pair of 0.
4. Special grids description end with a 0. The negative ones are stop-one-turn grid and positive ones are throw-again grid.
5. Then following another game description with the number of players.
6. The game descriptions end with a zero number of players.

Sample input:
3 6 3 2 5 1 3 4 2 3 1 2 0
2
6 95
99 1
0 0
-3
98
0
2
3 99
6 90
0 0
0
0
Sample output:
2
2
*/

#include "batch_read_number.c"
#include <stdbool.h>

#ifdef ONLINE_JUDGE
    #define debug_p //
    #define debug_header //
#else
    #include <time.h>
    #define debug_p printf
    #define debug_header() printf("=== Below line is debug output. >>>>\n")
#endif

#define GRID_COUNT 100

typedef struct {
    // 不论梯子或滑槽，目标格子只能有一个，否则违背了唯一性。
    int target;
    // 不论停一手或多一手，一个格子只能是其中之一。
    int mark;
} Grid;

int read_dice_numbers(int dice_numbers[])
{
    int input;
    int input_count = 0;
    for (; ; ++input_count) {
        scanf("%d", &input);
        if (0 == input) {
            return input_count;
        } else {
            dice_numbers[input_count] = input;
        }
    }
    return input_count;
}

void initialize_grids(Grid grids[]) {
    // TODO
}

int main()
{
    int dice_numbers[1000];
    int const dice_count = read_dice_numbers(dice_numbers);

    debug_header();
    for (int i = 0; i < dice_count; ++i) {
        printf("%d ", dice_numbers[i]);
    }
    printf("\n");

    int player_count = 0;
    while (true) {
        scanf("%d", &player_count);
        if (0 == player_count) {
            break;
        }
    }

    printf("%d\n", 2);

    debug_p("%ld clock used.\n", clock());
    return 0;
}
