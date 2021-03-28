cmake_minimum_required( VERSION 3.10 )
project(python_wrapper
        VERSION 1.0.0
        DESCRIPTION "Robocomp dsr python wrapper library"
        LANGUAGES CXX )

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
    MESSAGE(STATUS "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
    SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
    MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)


find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PythonLibs REQUIRED)

add_library(pydsr SHARED
        python_api.cpp
        )


target_link_libraries(pydsr PRIVATE Qt5::Core Eigen3::Eigen dsr_api dsr_core dsr_gui ${PYTHON_LIBRARIES})
target_include_directories(pydsr PRIVATE $ENV{ROBOCOMP}/classes/   $ENV{ROBOCOMP}/include/ ${PYTHON_INCLUDE_DIRS} )
target_link_directories(pydsr PRIVATE  $ENV{ROBOCOMP}/lib)
set_target_properties(pydsr PROPERTIES
        CMAKE_CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
        Python_ADDITIONAL_VERSIONS 3.8)

target_compile_options( pydsr PRIVATE -g -fmax-errors=5 -std=c++20 )
install( TARGETS pydsr
        DESTINATION /opt/robocomp/lib/ )