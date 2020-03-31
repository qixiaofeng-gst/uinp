#include <iostream>
#include <stdlib.h>
#include <cstring>

#include "board.h"
#include "impl/board.h"
#include "coordinates-tool.h"
using qxf::gomoku::Board;
using qxf::gomoku::CoordinatesTool;
using qxf::gomoku::c_zero;
using qxf::gomoku::c_board_width;

const char c_exit_flag = 'x';
const char* const c_msg_invalid = " is invalid";
const char* const c_msg_occupied = " is occupied";
const char* const c_msg_empty = "";

char physic_board[16 * c_board_width + 1] = "\
0|1|2|3|4|5|6|7|8|9|a|b|c|d|e|f\n\
1|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
2|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
3|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
4|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
5|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
6|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
7|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
8|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
9|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
a|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
b|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
c|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
d|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
e|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n\
f|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_\n";
short logical_board[c_board_width][c_board_width];
short play_records[c_board_width * c_board_width + 1];

bool isValidCoordinates(char x, char y)
{
  return true;
}

void clearScreen()
{
  #ifdef __linux__
    system("printf \"\\033c\"");
  #else
    system("CLS");
  #endif
}

/*
TODO:
1. winner judgement
2. simplest AI
4. RL AI
3. self training AI

Done:
1. board model is built
2. basic input and output method is implemented
*/
int main()
{
  memset(logical_board, 0, sizeof(short) * c_board_width * c_board_width);
  memset(play_records, 0, sizeof(short) * (c_board_width * c_board_width + 1));
  const char* msg = c_msg_empty;
  CoordinatesTool& util = CoordinatesTool::instance();
  char inputX = c_zero, inputY = c_zero;
  int step = 0;
  while (false == (inputX == c_exit_flag || inputY == c_exit_flag)) {
    clearScreen();
    Board board;
    board.test();
    std::cout << physic_board << "Last: " << inputX << inputY << msg << ", put:";
    std::cin >> inputX >> inputY;
    util.parseCoordinates(inputX, inputY);
    if (util.isCoordinatesValid()) {
      if (logical_board[util.y()][util.x()] > 0) {
        msg = c_msg_occupied;
      } else {
        int playerFlag = ++step % 2;
        bool isFirst = playerFlag == 1;
        msg = c_msg_empty;
        physic_board[util.indexForPhysicBoard()] = isFirst ? 'O' : 'X';
        logical_board[util.y()][util.x()] = isFirst ? 1 : 2;
        play_records[0] = step;
        play_records[step] = util.x() + util.y() * c_board_width;
      }
    } else {
      msg = c_msg_invalid;
    }
  }
  std::cout << "Bye bye!" << std::endl;
  return 0;
}
