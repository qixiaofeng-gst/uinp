#ifndef BOARD_H
#define BOARD_H

#include <memory>

namespace qxf {
namespace gomoku {

class BoardImpl;

void test();

class Board {
  public:
    Board();
  private:
    std::unique_ptr<BoardImpl> impl;
};

}//end namespace gomoku
}//end namespace qxf

#endif
