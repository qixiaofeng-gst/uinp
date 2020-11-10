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

#include <stdbool.h>
#include <stdio.h>

#ifdef ONLINE_JUDGE
    #define debug_p(...) //
    #define debug_header //
#else
    #include <time.h>
    #define debug_p(...) printf("==== debug >>>: " __VA_ARGS__)
    #define debug_header() printf("=== Below line is debug output. >>>>\n")
#endif

// 将格子数设置成 101，留出 0 位置，使角标从 1 开始。
#define GRID_COUNT 101
#define MARK_MORE 1
#define MARK_STOP 2

typedef struct {
    // 不论梯子或滑槽，目标格子只能有一个，否则违背了唯一性。
    int target;
    // 不论停一手或多一手，一个格子只能是其中之一。
    int mark;
} Grid;

typedef struct {
    int grid_index;
    int delay_mark;
} Player;

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
    for (int i = 0; i < GRID_COUNT; ++i) {
        grids[i].target = 0;
        grids[i].mark = 0;
    }
}

void read_channels(Grid grids[]) {
    int start, end;
    while (true) {
        scanf("%d %d", &start, &end);
        if ((0 == start) || (0 == end)) {
            break;
        }
        grids[start].target = end;
    }
}

void read_special_grids(Grid grids[]) {
    int index;
    while (true) {
        scanf("%d", &index);
        if (0 == index) {
            break;
        } else if (0 > index) {
            grids[-index].mark = MARK_STOP;
        } else {
            grids[index].mark = MARK_MORE;
        }
    }
}

int play_turn(Grid grids[], int dice_numbers[], Player * const player, int * const dice_index) {
    return 100;  // TODO remove this line.
    if (1 == player->delay_mark) {
        player->delay_mark = 0;
        return 0;
    }
    int const dice_number = dice_numbers[(*dice_index)++];
    int stop_grid_index = player->grid_index + dice_number;
    if (stop_grid_index >= GRID_COUNT) {
        return 0;
    }
    int const end_mark = GRID_COUNT - 1;
    if (stop_grid_index == end_mark) {
        return end_mark;
    }
    int const target_grid_index = grids[stop_grid_index].target;
    if (target_grid_index > 0) {
        stop_grid_index = target_grid_index;
    }
    // TODO =======
    return stop_grid_index;
}

void play_game(Grid grids[], int dice_numbers[], int const player_count) {
    Player players[player_count];
    for (int i = 0; i < player_count; ++i) {
        players[i].grid_index = 1;
        players[i].delay_mark = 0;
    }
    int dice_index = 0;
    while (true) {
        for (int i = 0; i < player_count; ++i) {
            int const result = play_turn(grids, dice_numbers, &players[i], &dice_index);
            if ((GRID_COUNT - 1) == result) {
                // 游戏结束
                printf("%d\n", i + 1);
                return;
            } else {
                continue;
            }
        }
    }
}

int main()
{
    int dice_numbers[1000];
    Grid grids[GRID_COUNT];
    int const dice_count = read_dice_numbers(dice_numbers);

    int player_count = 0;
    while (true) {
        scanf("%d", &player_count);
        if (0 == player_count) {
            break;
        }

        initialize_grids(grids);
        read_channels(grids);
        read_special_grids(grids);
        play_game(grids, dice_numbers, player_count);
    }

    debug_p("%ld clock used.\n", clock());
    return 0;
}
