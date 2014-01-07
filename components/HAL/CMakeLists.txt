INCLUDE( ${RoboComp_SOURCE_DIR}/cmake/robocomp.cmake )

SET(laser_interfaces
     Laser;
     DifferentialRobot
     )
     

SET(laser_src
  laserComp/src/laserComp.cpp
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

#QT4_WRAP_CPP( laserComp_MOC_SOURCES ${laser_headers} )
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )

RoboComp_ADD_COMPONENT( laserComp "${laser_interfaces}" "${laser_headers}" ${laser_src} )
target_link_libraries(  laserComp ${QT_LIBRARIES} robocomp_innermodel -lIce -lIceUtil -L/opt/robocomp/lib ${OSG_LIBRARY} ${OSGUTIL_LIBRARY} ${OSGDB_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} -lqmat -L${ICEROOT}/lib/ -L${ROBOCOMP_ROOT}/classes -lIce -lIceUtil -lpthread -lIceStorm -lgsl -lblas -losgGA -losgText -losgDB -losgUtil -losg -losgViewer innermodel )