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

##Initialize Ice

ADD_DEFINITIONS("-DICE_CPP11_MAPPING")
FIND_PACKAGE(Ice REQUIRED COMPONENTS Ice++11)


##Initialize robocomp
ROBOCOMP_INITIALIZE($ENV{ROBOCOMP}/)
ROBOCOMP_IDSL_TO_ICE(DSRGetID)
ROBOCOMP_ICE_TO_SRC(DSRGetID)

##Find Eigen
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)


#SET(SPECIFIC_LIBS fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen)

#message(STATUS  "Qt libs ${QT_LIBRARIES}")
#message(STATUS "ICe libs ${Ice_LIBRARIES}")

FIND_PACKAGE(pybind11)


set(CMAKE_CXX_STANDARD 20)
add_definitions(-g -std=c++2a -L /opt/robocomp/lib )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


pybind11_add_module(pydsr python_api.cpp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} ${HEADERS})
set_target_properties(pydsr PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(pydsr PRIVATE pthread   /opt/robocomp/lib/libdsr_api.so /opt/robocomp/lib/libdsr_core.so /opt/robocomp/lib/libdsr_gui.so ${SPECIFIC_LIBS} ${QT_LIBRARIES})
install(TARGETS pydsr
        DESTINATION /opt/robocomp/lib/)