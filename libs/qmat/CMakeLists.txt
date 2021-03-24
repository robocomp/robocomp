cmake_minimum_required( VERSION 3.12 )
project(qmat
        VERSION 2021.2.1
        DESCRIPTION "RoboComp qmat library"
        LANGUAGES CXX )


find_package(Qt5 COMPONENTS Core REQUIRED)

add_library(qmat SHARED)
target_sources(qmat
        PRIVATE qfundamental.cpp
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
        PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include/qmat/QMatAll)
target_link_libraries(qmat
        PUBLIC
        Qt5::Core
        gsl
        )
target_include_directories(qmat
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)

add_library(Robocomp::qmat ALIAS qmat)