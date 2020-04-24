#include <iostream>

#include "coordinates-tool.hpp"
#include "impl/coordinates-tool.hpp"

namespace qxf {
namespace gomoku {

CoordinatesTool& CoordinatesTool::instance()
{
  static CoordinatesTool self;
  return self;
}

CoordinatesTool::CoordinatesTool():
  impl(new CoordinatesToolImpl())
{}

bool CoordinatesTool::isCoordinatesValid()
{
  return impl->isCoordinatesValid();
}

int CoordinatesTool::x()
{
  return impl->x();
}

int CoordinatesTool::y()
{
  return impl->y();
}

int CoordinatesTool::indexForPhysicBoard()
{
  return impl->indexForPhysicBoard();
}

void CoordinatesTool::parseCoordinates(char x, char y)
{
  return impl->parseCoordinates(x, y);
}

} //end of namespace gomoku
} //end of namespace qxf
