cmake_minimum_required(VERSION 2.10)
project( rcremote
        VERSION 2021.2.1
        DESCRIPTION "RoboComp rcremote tool"
        LANGUAGES CXX )

INCLUDE( ${CMAKE_SOURCE_DIR}/cmake/robocompLocal.cmake )
ROBOCOMP_IDSL_TO_ICE( CommonBehavior IMU IMUPub RCRemote )

INSTALL(PROGRAMS "rcremote.py" DESTINATION /opt/robocomp/bin RENAME rcremote )
INSTALL(PROGRAMS "rcremoteserver.py" DESTINATION /opt/robocomp/bin RENAME rcremoteserver )
INSTALL(PROGRAMS "rcremoteshell.sh" DESTINATION /opt/robocomp/bin RENAME rcremoteshell )



