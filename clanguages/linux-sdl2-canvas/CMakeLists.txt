cmake_minimum_required(VERSION 2.4)
project(simplest-canvas)

find_library(SDL2 REQUIRED)
find_library(GL REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TOOL Release)
endif (NOT CMAKE_BUILD_TYPE)

set(CMAKE_C_FLAGS "-std=c99 -Wall -Wextra")
set(CMAKE_C_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
message(STATUS "Using build type: ${CMAKE_BUILD_TYPE}")

file(GLOB canvas_sources "src/*.c")
list(REMOVE_ITEM canvas_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/gl_main.c)
file(GLOB to-test "src/*.c")
list(REMOVE_ITEM to-test ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
list(REMOVE_ITEM to-test ${CMAKE_CURRENT_SOURCE_DIR}/src/gl_main.c)
file(GLOB tests "test/*.c")

add_executable(
        gl_canvas
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gl_main.c
)
target_link_libraries(
        gl_canvas PRIVATE

        SDL2
        GL
        GLEW
)

add_executable(
        canvas
        ${canvas_sources}
)
target_link_libraries(
        canvas PRIVATE

        SDL2
        GL
        m
)
target_include_directories(
        canvas PRIVATE

        include
)

add_executable(
        unit-test

        ${tests}
        ${to-test}
        ../gomoku-c/test/testool.c
)
target_link_libraries(
        unit-test PRIVATE

        SDL2
        m
)
target_include_directories(
        unit-test PRIVATE

        include
        ../gomoku-c/include
)
