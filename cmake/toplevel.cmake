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

#ADD the components
SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(Red         "${Esc}[31m")

FOREACH(SUBDIR ${SUBDIRS})
    message(STATUS "${Red}Configuring Component ${SUBDIR} ${ColourReset}")
    IF(NOT EXISTS "${SUBDIR}/.ignore_comp")
        ADD_SUBDIRECTORY(${SUBDIR})
    ENDIF()
ENDFOREACH()
