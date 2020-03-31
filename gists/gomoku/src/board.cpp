#include <iostream>

#include "board.hpp"
#include "impl/board.hpp"

namespace qxf {
namespace gomoku {

Board::Board():
  impl(new BoardImpl())
{
  std::string output("Board constructed");
  std::cout << output << std::endl;
}

void Board::play(unsigned int x, unsigned int y)
{
  std::cout << "Play at " << x << "," << y << std::endl;
}

} //end of namespace gomoku
} //end of namespace qxf
