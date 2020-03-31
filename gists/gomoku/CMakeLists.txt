cmake_minimum_required(VERSION 3.10)
project(gomoku VERSION 0.1)

include_directories(include src)

add_library(board STATIC src/board.cpp)
add_library(coordinates-tool STATIC src/coordinates-tool.cpp)
add_executable(gomoku src/gomoku.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY lib/)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib/)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin/)

target_link_libraries(gomoku PUBLIC
  board
  coordinates-tool)
