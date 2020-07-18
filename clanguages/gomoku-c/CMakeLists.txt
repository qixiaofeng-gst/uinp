cmake_minimum_required(VERSION 2.4)
project(gomoku-c)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TOOL Release)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "-std=c99 -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
message(STATUS "Using build type: ${CMAKE_BUILD_TYPE}")

file(GLOB sources "src/*.c")
file(GLOB to_test "src/*.c")
list(REMOVE_ITEM to_test ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)

file(GLOB tests "test/*.c")

include_directories(include)

add_executable(test ${tests} ${to_test})
target_include_directories(test PRIVATE src)

add_executable(main ${sources})
