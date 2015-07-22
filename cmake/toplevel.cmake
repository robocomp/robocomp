#toplevel cmake for robocomp workspace

cmake_minimum_required(VERSION 2.8)

#make sure this is a valid rc workspace
if( NOT EXISTS "../.rc_workspace")
    message(FATAL_ERROR "This is not a Robocomp workspace")
endif()

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
        LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

set(WORKSPACE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "workspace source path")
get_filename_component(WORKSPACE_PATH "../" REALPATH CACHE)

#ADD the components
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(Red         "${Esc}[31m")
set(BoldRed     "${Esc}[1;31m")

FOREACH(SUBDIR ${SUBDIRS})
    IF(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}/IGNORE_COMP")
        message(STATUS "${BoldRed}Configuring Component ${SUBDIR} ${ColourReset}")
        ADD_SUBDIRECTORY(${SUBDIR})
    ENDIF()
ENDFOREACH()

#append the source directory in .rc_install when install target is made
add_custom_command(TARGET install POST_BUILD COMMAND echo -n ;"${CMAKE_CURRENT_SOURCE_DIR}" >> .rc_install WORKING_DIRECTORY ${RC_COMPONENT_INSTALL_PATH} )

