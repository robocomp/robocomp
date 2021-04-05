cmake_minimum_required( VERSION 3.10 )
project(dsr_gui
        VERSION 2021.3.1
        DESCRIPTION "Robocomp DSR gui library"
        LANGUAGES CXX )


find_package(Qt5 COMPONENTS Widgets OpenGL Xml REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppitertools)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

# TODO: look for a better way to get automoc getting over .h files
set(headers_to_moc
        include/dsr/gui/viewers/osg_3d_viewer/osg_3d_viewer.h
        include/dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h
        include/dsr/gui/viewers/graph_viewer/graph_viewer.h
        include/dsr/gui/viewers/graph_viewer/graph_node.h
        include/dsr/gui/viewers/graph_viewer/graph_edge.h
        include/dsr/gui/viewers/tree_viewer/tree_viewer.h
        include/dsr/gui/viewers/_abstract_graphic_view.h
        include/dsr/gui/dsr_gui.h
        )

add_library(dsr_gui SHARED )
add_library(Robocomp::dsr_gui ALIAS dsr_gui )

target_sources(dsr_gui
        PRIVATE
        dsr_gui.cpp
        viewers/osg_3d_viewer/osg_3d_viewer.cpp
        viewers/qscene_2d_viewer/qscene_2d_viewer.cpp
        viewers/graph_viewer/graph_viewer.cpp
        viewers/graph_viewer/graph_node.cpp
        viewers/graph_viewer/graph_edge.cpp
        viewers/tree_viewer/tree_viewer.cpp
        viewers/_abstract_graphic_view.cpp
        ${headers_to_moc}
        )


target_link_libraries(dsr_gui
        PUBLIC Qt5::Widgets Qt5::OpenGL Qt5::Xml
        PUBLIC Eigen3::Eigen
        PRIVATE fastcdr fastrtps
        PRIVATE osgDB OpenThreads
        PRIVATE qmat osgviewer
        PRIVATE Robocomp::dsr_core
        )

target_include_directories(dsr_gui
        PUBLIC
        # TODO: Don't like, try to fix
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../classes/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../api/include/>
        $<INSTALL_INTERFACE:>

        )

set_target_properties(dsr_gui
        PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON)

target_compile_options(dsr_gui PUBLIC -g -fmax-errors=5 -std=c++20 )

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS dsr_gui EXPORT dsr_guiConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

install(DIRECTORY include/dsr DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT dsr_guiConfig DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dsr_gui/ NAMESPACE Robocomp::)


# This makes the project importable from the build directory
export(TARGETS dsr_gui FILE dsr_guiConfig.cmake)
