cmake_minimum_required(VERSION 2.6)
PROJECT( rcis )

INCLUDE( CMakeListsSpecific.txt )

## Source files
#SET ( SOURCES
#	${SOURCES}
#	robotsimulatorcomp.cpp
#	genericworker.cpp
#	genericmonitor.cpp
#	cameraI.cpp
#	differentialrobotI.cpp
#	genericbaseI.cpp
#	omnirobotI.cpp
#	imuI.cpp
#	innermodelmanagerI.cpp
#	jointmotorI.cpp
#	laserI.cpp
#	rgbdI.cpp
#	displayI.cpp
#	touchsensorI.cpp
#	${CMAKE_SOURCE_DIR}/classes/rapplication/rapplication.cpp
#	${CMAKE_SOURCE_DIR}/classes/qlog/qlog.cpp
#)

# # # Header files
SET ( HEADERS
	${HEADERS}
	genericworker.h
	genericbaseI.h
	genericmonitor.h
)

SET(LIBS -L${CMAKE_BUILD_DIR} ${LIBS} )


# RoboComp
INCLUDE( ${CMAKE_SOURCE_DIR}/cmake/robocompLocal.cmake )
ROBOCOMP_INITIALIZE( ${CMAKE_SOURCE_DIR}/ )
ROBOCOMP_IDSL_TO_ICE( Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display)
ROBOCOMP_ICE_TO_SRC_FUNC( ICES_SOURCES Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display)
SET(ROBOCOMP_LIBS ${ROBOCOMP_LIBS} -lrobocomp_qmat )

#include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../../libs/innermodel/include )
#include_directories(${CMAKE_SOURCE_DIR}/libs/innermodel/include)
#include_directories(${CMAKE_SOURCE_DIR}/libs/qmat/include)
#include_directories(${CMAKE_SOURCE_DIR}/libs/osgviewer/include)



# OPENSG
INCLUDE (${CMAKE_SOURCE_DIR}/cmake/modules/opensg.cmake )

# Qt
INCLUDE (${CMAKE_SOURCE_DIR}/cmake/modules/qt.cmake )

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


# Specify construction and link process
#SET (EXECUTABLE_OUTPUT_PATH ../bin)
add_executable( rcis  )
target_sources(rcis
		PRIVATE
		${SPECIFIC_SOURCES}
		${ICES_SOURCES}
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
		displayI.cpp
		touchsensorI.cpp
		${CMAKE_SOURCE_DIR}/classes/rapplication/rapplication.cpp
		${CMAKE_SOURCE_DIR}/classes/qlog/qlog.cpp
		${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS}
		)
target_link_libraries( rcis PRIVATE Robocomp::innermodel  Robocomp::qmat Robocomp::osgviewer glut OpenThreads osgGA osgDB osgUtil osgText ${LIBS} ${STATIC_LIBS} ${QT_LIBRARIES} -lboost_system -lboost_thread ${Ice_LIBRARIES})
target_include_directories(rcis PUBLIC
		${CMAKE_CURRENT_BINARY_DIR}
		${PYTHON_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS}
		${CMAKE_SOURCE_DIR}/libs/innermodel/include
		${CMAKE_SOURCE_DIR}/libs/osgviewer/include
		${CMAKE_SOURCE_DIR}/libs/qmat/include
		)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rcis DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
