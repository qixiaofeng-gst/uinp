#ifndef BOARD_IMPL_H
#define BOARD_IMPL_H

namespace qxf {
namespace gomoku {

class BoardImpl {
public:
  BoardImpl():
    x(0)
  {}
  ~BoardImpl() = default;
private:
  int x;
};

} //end of namespace gomoku
} //end of namespace qxf

#endif // guard end for BOARD_IMPL_H
