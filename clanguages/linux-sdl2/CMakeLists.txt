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

add_executable(canvas ${sources})
target_link_libraries(canvas PRIVATE
        SDL2)
target_include_directories(canvas PRIVATE
        include)
