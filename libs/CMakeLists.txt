cmake_minimum_required( VERSION 3.10 )
project(robocomp_libs
        VERSION 2021.3.1
        DESCRIPTION "Robocomp libs"
        LANGUAGES CXX )

include(GNUInstallDirs)

SET( CMAKE_AUTOMOC ON )
SET( CMAKE_AUTOUIC ON )

add_subdirectory(qmat)

add_subdirectory(osgviewer)

add_subdirectory(innermodel)

#add_subdirectory(innermodel-python3)


# TEST app for the libs
add_executable(testlibs EXCLUDE_FROM_ALL)
target_sources(testlibs
        PRIVATE
        test.cpp
        )

target_link_libraries(testlibs
        PRIVATE
        innermodel
        osgviewer
        )