#ifndef COORDINATES_TOOL_IMPL_HPP
#define COORDINATES_TOOL_IMPL_HPP

namespace qxf {
namespace gomoku {

class CoordinatesToolImpl {
public:
  CoordinatesToolImpl():
    isValid(false),
    indexX(0),
    indexY(0)
  {}
  bool isCoordinatesValid()
  {
    return isValid;
  }
  int x()
  {
    return indexX;
  }
  int y()
  {
    return indexY;
  }
  int indexForPhysicBoard()
  {
    return indexX * 2 + indexY * c_board_width;
  }
  void parseCoordinates(char x, char y)
  {
    isValid = false;
    indexX = charToIndex(x);
    if (0 == indexX) {
      return;
    }
    indexY = charToIndex(y);
    isValid = indexY > 0;
  }

private:
  int charToIndex(char c)
  {
    if (c > c_zero && c <= nine) {
      return c - c_zero;
    } else if (c >= a && c <= f) {
      return c - a + 10;
    } else {
      return 0; // invalid index
    }
  }

  const char nine = '9';
  const char a = 'a';
  const char f = 'f';

  bool isValid;
  int indexX;
  int indexY;
};

} //end of namespace gomoku
} //end of namespace qxf

#endif // guard end for COORDINATES_TOOL_IMPL_HPP
