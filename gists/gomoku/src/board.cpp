#include <iostream>

#include "board.h"
#include "impl/board.h"

namespace qxf {
namespace gomoku {

void Board::test()
{
  std::string output("Test method invoked.");
  std::cout << output << std::endl;
}

Board::Board():
  impl(new BoardImpl())
{
  std::cout << "Board constructed" << std::endl;
}

} //end of namespace gomoku
} //end of namespace qxf
