cmake_minimum_required( VERSION 3.0 )
project(dsr_gui
        VERSION 1.0.0
        DESCRIPTION "Robocomp DSR gui library"
        LANGUAGES CXX )

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
    MESSAGE(STATUS "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
    SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
    MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)


find_package(Qt5 COMPONENTS Widgets OpenGL Xml REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppitertools)

add_library(dsr_gui SHARED
        dsr_gui.cpp
        viewers/osg_3d_viewer/osg_3d_viewer.cpp
        viewers/qscene_2d_viewer/qscene_2d_viewer.cpp
        viewers/graph_viewer/graph_viewer.cpp
        viewers/graph_viewer/graph_node.cpp
        viewers/graph_viewer/graph_edge.cpp
        viewers/tree_viewer/tree_viewer.cpp
        viewers/_abstract_graphic_view.cpp
        )


target_link_libraries(dsr_gui Qt5::Widgets Qt5::OpenGL Qt5::Xml fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen )
target_include_directories(dsr_gui PRIVATE $ENV{ROBOCOMP}/classes/ $ENV{ROBOCOMP}/include/ )

set_target_properties(dsr_gui PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON)

target_compile_options(dsr_gui PUBLIC -g -fmax-errors=5 -std=c++20 )
install(TARGETS dsr_gui
        LIBRARY DESTINATION /opt/robocomp/lib/
        )