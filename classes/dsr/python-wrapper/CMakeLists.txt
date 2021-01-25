cmake_minimum_required(VERSION 2.8.11)
project(python_wrapper)

##Find robocomp

IF ("$ENV{ROBOCOMP}" STREQUAL "")
    MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
    SET(ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ("$ENV{ROBOCOMP}" STREQUAL "")

IF (NOT EXISTS $ENV{ROBOCOMP}/cmake)
    MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF (NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE($ENV{ROBOCOMP}/cmake/robocomp.cmake)
INCLUDE($ENV{ROBOCOMP}/cmake/modules/qt.cmake)


##Initialize robocomp
ROBOCOMP_INITIALIZE($ENV{ROBOCOMP}/)

set(Python_ADDITIONAL_VERSIONS 3.8)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

FIND_PACKAGE(pybind11)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-g -std=c++2a -L /opt/robocomp/lib )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(/usr/include/eigen3)
pybind11_add_module(pydsr python_api.cpp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} ${HEADERS})
set_target_properties(pydsr PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(pydsr PRIVATE pthread  ${CMAKE_CURRENT_BINARY_DIR}/../api/libdsr_api.so ${CMAKE_CURRENT_BINARY_DIR}/../core/libdsr_core.so ${CMAKE_CURRENT_BINARY_DIR}/../gui/libdsr_gui.so ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${PYTHON_LIBRARIES} )
install(TARGETS pydsr
        DESTINATION /opt/robocomp/lib/)
