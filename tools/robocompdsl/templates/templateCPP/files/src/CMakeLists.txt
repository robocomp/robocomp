cmake_minimum_required(VERSION 2.8.11)


IF ( "$$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
  SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $$ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $$ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $$ENV{ROBOCOMP}/cmake)

INCLUDE( $$ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE( $$ENV{ROBOCOMP}/cmake/modules/qt.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  $${SOURCES}
  $$ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $$ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $$ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  ${interface_sources}
  ${statemachine_visual_sources}
)

${ros_includes}

${cpp11_ice_packages}

${agm_includes}

#ROBOCOMP
ROBOCOMP_INITIALIZE( $$ENV{ROBOCOMP}/ )
${wrap_ice}
SET (EXECUTABLE_OUTPUT_PATH $${RC_COMPONENT_DEVEL_PATH}/bin)

${wrap_ui}

# Specify construction and link process
ADD_EXECUTABLE( ${component_name} $${SOURCES} $${MOC_SOURCES} $${RC_SOURCES} $${UI_HEADERS} )
TARGET_LINK_LIBRARIES( ${component_name} $${LIBS} $${STATIC_LIBS} $${SPECIFIC_LIBS} $${QT_LIBRARIES} $${Ice_LIBRARIES})
INSTALL(FILES $${EXECUTABLE_OUTPUT_PATH}/${component_name} DESTINATION $${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
