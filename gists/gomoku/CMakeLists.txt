cmake_minimum_required(VERSION 3.10)
project(gomoku VERSION 0.1)

add_library(board STATIC src/board.cpp)
add_executable(gomoku src/gomoku.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

target_link_libraries(gomoku PUBLIC board)