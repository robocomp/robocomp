#toplevel cmake for robocomp workspace

cmake_minimum_required(VERSION 2.8)

#make sure this is a valid rc workspace
if( NOT EXISTS "../.rc_workspace")
    message(FATAL_ERROR "This is not a Robocomp workspace")
endif()

#ADD the components
FOREACH(SUBDIR ${SUBDIRS})
    IF(NOT EXISTS "${SUBDIR}/.ignore_comp")
        ADD_SUBDIRECTORY(${SUBDIR})
        message(STATUS "ADDING COMPONENT ${SUBDIR}")
    ENDIF()
ENDFOREACH()

#for the time being we are not shifting executables into devel