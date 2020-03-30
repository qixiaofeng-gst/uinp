#ifndef COORDINATES_TOOL_H_
#define COORDINATES_TOOL_H_

#include <memory>

namespace qxf {
namespace gomoku {

const char c_zero = '0';
const int c_board_width = 32;

class CoordinatesToolImpl;

class CoordinatesTool {
public:
  static CoordinatesTool& instance();

  bool isCoordinatesValid();
  int x();
  int y();
  int indexForPhysicBoard();
  void parseCoordinates(char x, char y);

private:
  CoordinatesTool();
  ~CoordinatesTool() = default;
  std::unique_ptr<CoordinatesToolImpl> impl;
};

} //end of namespace gomoku
} //end of namespace qxf

#endif
