cmake_minimum_required( VERSION 3.10 )
project(dsr_api
        VERSION 2021.2.1
        DESCRIPTION "RoboComp InnerModel library"
        LANGUAGES CXX )


SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(cppitertools)

set(headers_to_moc
        include/dsr/api/dsr_api.h
        include/dsr/api/dsr_inner_eigen_api.h
        )

add_library(dsr_api SHARED )
add_library(Robocomp::dsr_api ALIAS dsr_api )


target_sources(dsr_api
        PRIVATE
        dsr_api.cpp
        dsr_camera_api.cpp
        dsr_inner_eigen_api.cpp
        dsr_rt_api.cpp
        dsr_utils.cpp
        ${headers_to_moc}
        )


target_link_libraries(dsr_api
        PRIVATE
        fastcdr fastrtps
        osgDB OpenThreads
        Qt5::Core
        Robocomp::qmat
        PUBLIC
        Eigen3::Eigen
        )

target_include_directories(dsr_api
        PUBLIC
        # Headers of DSR Core
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../qmat/include/>
        # TODO: Don't like, try to fix
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../classes/>
        # Own include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
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

install(DIRECTORY include/dsr DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT dsr_apiConfig DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dsr_api/  NAMESPACE Robocomp::)

# This makes the project importable from the build directory
export(TARGETS dsr_api FILE dsr_apiConfig.cmake)