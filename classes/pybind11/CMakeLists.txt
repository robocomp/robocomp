cmake_minimum_required(VERSION 3.4)
project(example)

find_package(pybind11 REQUIRED)  # or add_subdirectory(pybind11)

set(CMAKE_CXX_STANDARD 20)

add_executable(example main.cpp)
target_link_libraries(example PRIVATE pybind11::embed)
