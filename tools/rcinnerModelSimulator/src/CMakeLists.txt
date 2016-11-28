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
	genericbaseI.cpp
	omnirobotI.cpp
	imuI.cpp
	innermodelmanagerI.cpp
	jointmotorI.cpp
	laserI.cpp
	rgbdI.cpp
	touchsensorI.cpp
	$ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
	$ENV{ROBOCOMP}/classes/qlog/qlog.cpp
)

# Header files
SET ( HEADERS
	${HEADERS}
	genericworker.h
	genericbaseI.h
	genericmonitor.h
	cameraI.h
	differentialrobotI.h
	omnirobotI.h
	imuI.h
	innermodelmanagerI.h
	jointmotorI.h
	laserI.h
	rgbdI.h
	touchsensorI.h
)

SET(LIBS -L${CMAKE_BUILD_DIR} ${LIBS} )


# RoboComp
INCLUDE( $ENV{ROBOCOMP}/cmake/robocompLocal.cmake )
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase )
SET(ROBOCOMP_LIBS ${ROBOCOMP_LIBS} -lrobocomp_qmat )

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../../libs/innermodel/include )
include_directories($ENV{ROBOCOMP}/libs/innermodel/include)
include_directories($ENV{ROBOCOMP}/libs/qmat/include)
include_directories($ENV{ROBOCOMP}/libs/osgviewer/include)


# IPP
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/ipp.cmake )


# OPENSG
INCLUDE ($ENV{ROBOCOMP}/cmake/modules/opensg.cmake )

# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB --std=c++11)
FIND_PACKAGE( Qt4 REQUIRED )
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
SET( QT_USE_QTXML TRUE )
SET( QT_USE_QTSTATE TRUE )
SET( QT_USE_QTSTATEMACHINE TRUE )
INCLUDE( ${QT_USE_FILE} )
QT4_WRAP_CPP( MOC_SOURCES ${HEADERS} )
QT4_WRAP_UI( UI_HEADERS guiDlg.ui )



# Python bindings
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
if (PYTHON_BINDINGS_SUPPORT)
	# Add bindings source code file
	SET(srcs ${srcs} innermodel_python.cpp)
	find_package(Boost COMPONENTS system thread python REQUIRED)
	find_package(PythonLibs REQUIRED)
	MESSAGE(STATUS "im PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
	MESSAGE(STATUS "im PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS}")
	link_directories(${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS})
endif (PYTHON_BINDINGS_SUPPORT)

include_directories( ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

# Specify construction and link process
# SET (EXECUTABLE_OUTPUT_PATH ../bin)
ADD_EXECUTABLE( rcis ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( rcis glut OpenThreads osgGA osgDB osgUtil osgText ${LIBS} ${STATIC_LIBS} ${QT_LIBRARIES} -lboost_system -lboost_thread )
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/rcis DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )









