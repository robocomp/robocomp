cmake_minimum_required( VERSION 3.10 )
project(dsr_api
        VERSION 2021.2.1
        DESCRIPTION "RoboComp InnerModel library"
        LANGUAGES CXX )



find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppitertools)

add_library(dsr_api SHARED )
add_library(Robocomp::dsr_api ALIAS dsr_api )

target_sources(dsr_api
        PRIVATE
        dsr_api.cpp
        dsr_camera_api.cpp
        dsr_inner_eigen_api.cpp
        dsr_rt_api.cpp
        dsr_utils.cpp
        )


target_link_libraries(dsr_api
        PRIVATE
        fastcdr osgDB OpenThreads
        PUBLIC
        Eigen3::Eigen
        fastrtps
        Qt5::Core
        )

MESSAGE(STATUS "###############${CMAKE_SOURCE_DIR}/classes/")
target_include_directories(dsr_api
        PUBLIC
        # TODO: Don't like, try to fix
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../classes/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
        $<INSTALL_INTERFACE:>
        )


set_target_properties(dsr_api PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON )

target_compile_options(dsr_api PUBLIC -g -fmax-errors=5 -std=c++20 )


# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS dsr_api EXPORT dsr_apiConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

install(FILES dsr_api.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dsr/api/)

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT dsr_apiConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS dsr_api FILE dsr_apiConfig.cmake)