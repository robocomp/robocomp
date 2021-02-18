cmake_minimum_required( VERSION 3.12 )
project(robocomp_libs
        VERSION 2021.2.1
        DESCRIPTION "Robocomp libs"
        LANGUAGES CXX )

add_subdirectory(qmat)

add_subdirectory(osgviewer)

add_subdirectory(innermodel)

#add_subdirectory(innermodel-python3)