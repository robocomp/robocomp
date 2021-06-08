cmake_minimum_required(VERSION 3.10)


IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
  SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/qt.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
)




#ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_IDSL_TO_ICE( CommonBehavior )
ROBOCOMP_ICE_TO_SRC( CommonBehavior )
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

QT_WRAP_UI( UI_HEADERS mainUI.ui )

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-g  -fmax-errors=1 -fno-char8_t)

# Specify construction and link process
ADD_EXECUTABLE( testcomp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( testcomp ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES})
SET_TARGET_PROPERTIES(testcomp
        PROPERTIES
        CXX_STANDARD 20
        )
INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/testcomp DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
