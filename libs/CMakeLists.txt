cmake_minimum_required( VERSION 3.10 )
project(robocomp_libs
        VERSION 2021.3.1
        DESCRIPTION "Robocomp libs"
        LANGUAGES CXX )

include(GNUInstallDirs)

add_subdirectory(qmat)

add_subdirectory(osgviewer)

add_subdirectory(innermodel)

# TEST app for the libs
add_executable(testlibs EXCLUDE_FROM_ALL
        test_libs.cpp
        )
target_sources(testlibs
        PRIVATE
        test_libs.cpp
        )

target_link_libraries(testlibs
        PRIVATE
        Robocomp::innermodel
        Robocomp::osgviewer
        )

#add_subdirectory(innermodel-python3)





