cmake_minimum_required( VERSION 3.10 )
project(dsr_api
        VERSION 2021.2.1
        DESCRIPTION "RoboComp InnerModel library"
        LANGUAGES CXX )

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
    MESSAGE(STATUS "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
    SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
    MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)


find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppitertools)

add_library(dsr_api SHARED
        dsr_api.cpp
        dsr_camera_api.cpp
        dsr_inner_eigen_api.cpp
        dsr_rt_api.cpp
        dsr_utils.cpp
        )


target_link_libraries(dsr_api  Qt5::Core fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen )
target_include_directories(dsr_api
        PRIVATE $ENV{ROBOCOMP}/classes/ $ENV{ROBOCOMP}/include/
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


set_target_properties(dsr_api PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON )

target_compile_options(dsr_api PUBLIC -g -fmax-errors=5 -std=c++20 )
install(TARGETS dsr_api
        LIBRARY DESTINATION /opt/robocomp/lib/
        )