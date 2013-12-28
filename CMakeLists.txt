cmake_minimum_required( VERSION 2.6 FATAL_ERROR )

SET_PROPERTY(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

project(RoboComp)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# Add the framework
include( ${RoboComp_SOURCE_DIR}/cmake/robocomp_targets.cmake )

set( RoboComp_BIN_INSTALL_DIR "/opt/robocomp/bin" )
set( RoboComp_BIN_INSTALL_DIR "/opt/robocomp/lib" )

#set robocomp version
set( RoboComp_VERSION 1.0 CACHE STRING "PCL version")

SUBDIRS(
	classes
	)

