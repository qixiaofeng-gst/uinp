cmake_minimum_required(VERSION 2.4)
project(qxf-canvas)

find_library(SDL2 REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TOOL Release)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "-std=c99 -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
message(STATUS "Using build type: ${CMAKE_BUILD_TYPE}")

file(GLOB sources "src/*.c")
file(GLOB to-test "src/*.c")
list(REMOVE_ITEM to-test ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
file(GLOB tests "test/*.c")

add_executable(canvas ${sources})
target_link_libraries(canvas PRIVATE
        SDL2)
target_include_directories(canvas PRIVATE
        include)

add_executable(unit-test
        ${tests}
        ${to-test}
        ../gomoku-c/test/testool.c)
target_link_libraries(unit-test PRIVATE
        SDL2)
target_include_directories(unit-test PRIVATE
        include
        ../gomoku-c/include)
