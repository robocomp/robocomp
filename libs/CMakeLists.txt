cmake_minimum_required( VERSION 3.10 )
project(robocomp_libs
        VERSION 2021.2.1
        DESCRIPTION "Robocomp libs"
        LANGUAGES CXX )

include(GNUInstallDirs)

SET( CMAKE_AUTOMOC ON )
SET( CMAKE_AUTOUIC ON )

add_subdirectory(qmat)

add_subdirectory(osgviewer)

add_subdirectory(innermodel)

#add_subdirectory(innermodel-python3)
add_executable(testlibs)
target_sources(testlibs
        PRIVATE
        test.cpp
        )
target_link_libraries(testlibs
        PRIVATE
        innermodel
        osgviewer)