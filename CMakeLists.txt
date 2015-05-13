# RoboComp global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(RoboComp)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#Set RoboComp directories
set(RoboComp_LIBS_DIR ${RoboComp_SOURCE_DIR}/libs)
set(RoboComp_CLASSES_DIR ${RoboComp_SOURCE_DIR}/classes)
set(RoboComp_INTERFACES_DIR ${RoboComp_SOURCE_DIR}/interfaces)
# Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

# Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()

# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

include(${RoboComp_SOURCE_DIR}/cmake/robocomp_targets.cmake)

# ---[ Unix/Darwin/Windows specific flags
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -Wconversion -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof")
#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O4")

include(${RoboComp_SOURCE_DIR}/cmake/robocomp_utils.cmake)
set(RoboComp_VERSION 1.0 CACHE STRING "RoboComp version")
SET_INSTALL_DIRS()

# set(RoboComp_OUTPUT_LIB_DIR ${RoboComp_BINARY_DIR}/libs)
# set(RoboComp_OUTPUT_BIN_DIR ${RoboComp_BINARY_DIR}/bin)
# make_directory(${RoboComp_OUTPUT_LIB_DIR})
# make_directory(${RoboComp_OUTPUT_BIN_DIR})
#
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${RoboComp_OUTPUT_LIB_DIR}")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${RoboComp_OUTPUT_BIN_DIR}")
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${RoboComp_OUTPUT_LIB_DIR}")

# Add an "uninstall" target
configure_file("${RoboComp_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
               "${RoboComp_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                  "${RoboComp_BINARY_DIR}/uninstall_target.cmake")

# RoboComp global dependencies
find_package(OpenMP REQUIRED) ## search OpenMP libs
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
else(OPENMP_FOUND)
  message (STATUS "Not found OpenMP")
endif()


# Python binding
option (PYTHON_BINDINGS_SUPPORT "Build with Python bindings support" OFF)
if (PYTHON_BINDINGS_SUPPORT)
	MESSAGE(STATUS "PYTHON BINDINGS:        YES")
	add_definitions("-DPYTHON_BINDINGS_SUPPORT")
else (PYTHON_BINDINGS_SUPPORT)
	MESSAGE(STATUS "PYTHON BINDINGS:        NO")
endif (PYTHON_BINDINGS_SUPPORT)


# Find QT4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
find_package(Qt4 REQUIRED)
if (QT4_FOUND)
  include(${QT_USE_FILE})
  add_definitions( "-DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB" )
endif (QT4_FOUND)


#Find Doxygen and html help compiler if any
find_package(Doxygen)
if(DOXYGEN_FOUND)
  find_package(HTMLHelp)
endif(DOXYGEN_FOUND)

SUBDIRS(
	libs
	tools/robocompdsl
	tools/rcinnerModelSimulator
	tools/rcmanager
	tools/rcmonitor
	tools/rcremote
)


add_custom_target(mrproper
  COMMAND ${CMAKE_MAKE_PROGRAM} clean
   COMMAND find $ENV{ROBOCOMP} -name CMakeFiles -exec rm -rf {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name CMakeCache.txt -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name cmake_install.cmake -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name Makefile -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name *.kd* -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name *_moc* -exec rm {} "\\;"
   COMMAND find $ENV{ROBOCOMP} -name Makefile -exec rm {} "\\;"
)

#For packaging
include(${RoboComp_SOURCE_DIR}/cmake/package_details.cmake)

#packaging
include(CPack)