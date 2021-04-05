cmake_minimum_required(VERSION 3.10)
project(dsr
        VERSION 2021.3.1
        DESCRIPTION "Robocomp DSR library"
        LANGUAGES CXX)

include(GNUInstallDirs)

add_subdirectory(api)
add_subdirectory(core)
add_subdirectory(gui)
#add_subdirectory(python-wrapper)

