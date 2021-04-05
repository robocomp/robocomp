cmake_minimum_required( VERSION 3.10 )
project( dsr_core
        VERSION 1.0.0
        DESCRIPTION "Robocomp DSR core library"
        LANGUAGES CXX )


find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(dsr_core SHARED)
add_library(Robocomp::dsr_core ALIAS dsr_core)

target_sources(dsr_core
        PRIVATE
        types/crdt_types.cpp
        types/user_types.cpp
        rtps/dsrpublisher.cpp
        rtps/dsrsubscriber.cpp
        rtps/dsrparticipant.cpp
        topics/IDLGraphPubSubTypes.cxx
        topics/IDLGraph.cxx
        id_generator.cpp
        )


target_link_libraries(dsr_core PUBLIC Qt5::Core fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen)


target_include_directories(dsr_core
        PUBLIC
        # TODO: Don't like, try to fix
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../classes/>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:>
        )

set_target_properties(dsr_core PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON)

target_compile_options(dsr_core PUBLIC -g -fmax-errors=5 -std=c++20 )

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS dsr_core EXPORT dsr_coreConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        )

install(DIRECTORY include/dsr DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT dsr_coreConfig DESTINATION share/robocomp/cmake)

# This makes the project importable from the build directory
export(TARGETS dsr_core FILE dsr_coreConfig.cmake)