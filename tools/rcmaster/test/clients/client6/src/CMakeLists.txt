cmake_minimum_required(VERSION 2.8.11)
PROJECT(  client4 )

 
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
  asrI.cpp
)



#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior Test ASR )
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
SET( CMAKE_AUTOMOC TRUE )
INCLUDE( ${QT_USE_FILE} )

# Specify construction and link process
ADD_EXECUTABLE( client4 ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( client4 ${QT_LIBRARIES} ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS})
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/client4 DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
