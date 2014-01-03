cmake_minimum_required(VERSION 2.6)
PROJECT( laserComp )

# Gazebo
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/gazebo.cmake )
IF (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo not found. Gazebo's laser interface will NOT be supported!\n\n")
ELSE (GAZEBO_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nGazebo found. Gazebo's laser interface WILL be supported.\n\n")
ENDIF (GAZEBO_FOUND EQUAL 0)

# Player
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/player.cmake )
IF (PLAYER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nPlayer handler will NOT be built!\n\n")
ELSE (PLAYER_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nPlayer handler WILL be built.\n\n")
ENDIF (PLAYER_FOUND EQUAL 0)

# lURG
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/lurg.cmake )
IF (LURG_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nlurg not found. urg's laser interface will NOT be supported!\n\n")
ELSE (LURG_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nlurg found. urg's laser interface WILL be supported.\n\n")
ENDIF (LURG_FOUND EQUAL 0)

# lSICK
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/sick.cmake )
IF (LSICK_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nsick not found. sick's laser interface will NOT be supported!\n\n")
ELSE (LSICK_FOUND EQUAL 0)
  MESSAGE(STATUS "\n\nsick found. sick's laser interface WILL be supported.\n\n")
ENDIF (LSICK_FOUND EQUAL 0)

# Sources set
SET ( SOURCES
  laserComp.cpp
  laserI.cpp
  gazebohandler.cpp
  playerhandler.cpp
  hokuyogenerichandler.cpp
  hokuyohandler.cpp
  sickhandler.cpp
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/q4serialport/q4serialport.cpp
)

# Headers set
SET ( HEADERS
  laserI.h
  generichandler.h
  hokuyogenerichandler.h
  hokuyohandler.h
  sickhandler.h
  $ENV{ROBOCOMP}/classes/q4serialport/q4serialport.h
)

# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( Laser DifferentialRobot )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( laserComp ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( laserComp ${QT_LIBRARIES} ${LIBS} )
INSTALL(FILES laserComp DESTINATION ${BIN_INSTALL_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
