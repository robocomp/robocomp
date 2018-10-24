
cmake_minimum_required(VERSION 2.6)
PROJECT(  rclogger )

 
IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
  SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  specificmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  specificworker.cpp
  loggerI.cpp
)



#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior Logger )
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Qt
include(${RoboComp_SOURCE_DIR}/cmake/modules/qt.cmake)


# Specify construction and link process
ADD_EXECUTABLE( rclogger ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( rclogger ${QT_LIBRARIES} ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/rclogger DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
