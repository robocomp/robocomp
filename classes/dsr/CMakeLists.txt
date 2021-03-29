cmake_minimum_required(VERSION 3.0)

project(
        dsr
        VERSION 1.0.0
        DESCRIPTION "Robocomp DSR library"
        LANGUAGES CXX)

add_subdirectory(api)
add_subdirectory(core)
add_subdirectory(gui)
add_subdirectory(python-wrapper)