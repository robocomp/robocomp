cmake_minimum_required( VERSION 2.8 )

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path" FORCE)
SET(RC_COMPONENT_DEVEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "component binary path" FORCE)
SET(RC_COMPONENT_INSTALL_PATH "/opt/robocomp" CACHE STRING "component install path")

SUBDIRS( src )

INSTALL(FILES etc/config DESTINATION ${RC_COMPONENT_INSTALL_PATH}/etc-default/ RENAME agmmission.conf )