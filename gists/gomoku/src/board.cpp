#include <iostream>

#include "../include/board.h"

namespace qxf {
namespace gomoku {

class BoardImpl {
public:
private:
  int x;
};

void test()
{
  std::string output("Test method invoked.");
  std::cout << output << std::endl;
}

void Board::test()
{
  std::cout << "Board::test invoked" << std::endl;
}

Board::Board():
  impl(new BoardImpl())
{
  std::cout << "Board constructed" << std::endl;
}

} //end of namespace gomoku
} //end of namespace qxf
