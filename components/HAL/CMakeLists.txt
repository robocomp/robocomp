INCLUDE( ${RoboComp_SOURCE_DIR}/cmake/robocomp.cmake )


SET(laser_src
  laserComp/src/laserI.cpp
  laserComp/src/gazebohandler.cpp
  laserComp/src/playerhandler.cpp
  laserComp/src/hokuyogenerichandler.cpp
  laserComp/src/sickhandler.cpp
  ${RoboComp_SOURCE_DIR}/classes/rapplication/rapplication.cpp
  ${RoboComp_SOURCE_DIR}/classes/q4serialport/q4serialport.cpp 
  )
  
SET (laser_headers
  laserComp/src/laserI.h
  laserComp/src/generichandler.h
  laserComp/src/hokuyogenerichandler.h
  laserComp/src/hokuyohandler.h
  laserComp/src/sickhandler.h
  ${RoboComp_SOURCE_DIR}/classes/q4serialport/q4serialport.h
)

QT4_WRAP_CPP( MOC_SOURCES ${laser_headers} )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( Laser DifferentialRobot )

RoboComp_ADD_EXECUTABLE( laserComp  ${laser_src} ${MOC_SOURCES} )
target_link_libraries( laserComp ${QT_LIBRARIES} robocomp_innermodel -lIce -lIceUtil -L/opt/robocomp/lib ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} -lqmat -L${ICEROOT}/lib/ -L${ROBOCOMP_ROOT}/classes -lIce -lIceUtil -lpthread -lIceStorm -lgsl -lblas ${QT_LIBRARIES} -losgGA -losgText -losgDB -losgUtil -losg -losgViewer innermodel )