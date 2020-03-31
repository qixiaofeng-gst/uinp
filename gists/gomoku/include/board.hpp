#ifndef BOARD_H
#define BOARD_H

#include <memory>

namespace qxf {
namespace gomoku {

class BoardImpl;
class Board {
public:
  Board();
  void play(unsigned int x, unsigned int y);
private:
  std::unique_ptr<BoardImpl> impl;
};

}//end namespace gomoku
}//end namespace qxf

#endif
