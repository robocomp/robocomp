cmake_minimum_required( VERSION 3.0 )
project( dsr_core
        VERSION 1.0.0
        DESCRIPTION "Robocomp DSR core library"
        LANGUAGES CXX )


find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(dsr_core SHARED
        types/crdt_types.cpp
        types/user_types.cpp
        rtps/dsrpublisher.cpp
        rtps/dsrsubscriber.cpp
        rtps/dsrparticipant.cpp
        topics/IDLGraphPubSubTypes.cxx
        topics/IDLGraph.cxx
        id_generator.cpp )


target_link_libraries(dsr_core PUBLIC Qt5::Core fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen)
set_target_properties(dsr_core PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON)

target_compile_options(dsr_core PUBLIC -g -fmax-errors=5 -std=c++20 )
install(TARGETS dsr_core
        LIBRARY DESTINATION /opt/robocomp/lib/
        )