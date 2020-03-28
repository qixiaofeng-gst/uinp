#ifndef COORDINATES_TOOL_H_
#define COORDINATES_TOOL_H_

namespace qxf {
namespace gomoku {

const char c_zero = '0';
const int c_board_width = 32;

class CoordinatesTool {
public:
  static CoordinatesTool& instance()
  {
    static CoordinatesTool self;
    return self;
  }
  inline bool isCoordinatesValid()
  {
    return isValid;
  }
  inline int x()
  {
    return indexX;
  }
  inline int y()
  {
    return indexY;
  }
  inline int indexForPhysicBoard()
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
  CoordinatesTool():
    isValid(false),
    indexX(0),
    indexY(0)
  {}
  ~CoordinatesTool() = default;
  inline int charToIndex(char c)
  {
    if (c > c_zero && c <= nine) {
      return c - c_zero;
    } else if (c >= a && c <= f) {
      return c - a + 10;
    } else {
      return 0; // invalid index
    }
  }

private:
  const char nine = '9';
  const char a = 'a';
  const char f = 'f';

  bool isValid;
  int indexX;
  int indexY;
};

} //end of namespace gomoku
} //end of namespace qxf

#endif
