cmake_minimum_required(VERSION 3.21)
project(cells)

set(CMAKE_CXX_STANDARD 23)

file(GLOB_RECURSE imgui_Sources
        imgui/*.cpp imgui/*.h)

find_package(glfw3)

SET(CMAKE_CXX_FLAGS "-D_LINUX -O0 -g -Wall -Wno-unknown-pragmas -Werror=return-type ")

include_directories(imgui include)
add_executable(cells main.cpp myutils.hpp world.hpp world.cpp include/GL/gl3w.c shader.hpp shader.cpp api.hpp explosion.hpp ${imgui_Sources} MyStrategy.hpp MyStrategy.cpp)
target_link_libraries(cells glfw dl GL pthread X11)
