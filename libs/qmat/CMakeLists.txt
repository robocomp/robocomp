cmake_minimum_required( VERSION 3.10 )
project(qmat
        VERSION 2021.2.1
        DESCRIPTION "RoboComp qmat library"
        LANGUAGES CXX )

find_package(Qt5 COMPONENTS Core REQUIRED)

add_library(qmat SHARED)

target_sources(qmat
        PRIVATE
        qfundamental.cpp
        qessential.cpp
        qcamera.cpp
        qextrinsics.cpp
        qmat.cpp
        qmovingrobot.cpp
        qvec.cpp
        qmatrot.cpp
        qrtmat.cpp
        qhomo.cpp
        qline2d.cpp
        quaternion.cpp
        )
target_link_libraries(qmat
        PUBLIC
        Qt5::Core
        gsl
        )

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(qmat PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS qmat EXPORT qmatConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT qmatConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS qmat FILE qmatConfig.cmake)