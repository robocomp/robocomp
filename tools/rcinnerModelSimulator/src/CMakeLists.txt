cmake_minimum_required(VERSION 2.6)
PROJECT( rcis )

INCLUDE( CMakeListsSpecific.txt )

# Source files
SET ( SOURCES
	${SOURCES}
	robotsimulatorcomp.cpp
	genericworker.cpp
	genericmonitor.cpp
	cameraI.cpp
	differentialrobotI.cpp
	imuI.cpp
	innermodelmanagerI.cpp
	jointmotorI.cpp
	laserI.cpp
	rgbdI.cpp
	$ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
	$ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Header files
SET ( HEADERS
	${HEADERS}
	genericworker.h
	genericmonitor.h
	cameraI.h
	differentialrobotI.h
	imuI.h
	innermodelmanagerI.h
	jointmotorI.h
	laserI.h
	rgbdI.h
)


# Robocomp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( Camera CommonBehavior CommonHead DifferentialRobot IMU InnerModelManager JointMotor Laser RGBD )
ROBOCOMP_LIBRARY( qmat )


# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )


# OPENSG
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/opensg.cmake )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )
QT4_WRAP_UI( UI_HEADERS guiDlg.ui )

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( rcis ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( rcis glut OpenThreads osgGA osgDB osgUtil osgText ${LIBS} ${STATIC_LIBS} ${QT_LIBRARIES} -lrobocomp_osgviewer)
INSTALL(FILES ${CMAKE_BINARY_DIR}/bin/rcis DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
