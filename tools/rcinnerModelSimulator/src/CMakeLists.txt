cmake_minimum_required(VERSION 3.10)


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
#SET ( HEADERS
#	${HEADERS}
#	genericworker.h
#	genericbaseI.h
#	genericmonitor.h
#)
#
#SET(LIBS -L${CMAKE_BUILD_DIR} ${LIBS} )


## RoboComp
INCLUDE( ${CMAKE_SOURCE_DIR}/cmake/robocomp_build.cmake )
#ROBOCOMP_INITIALIZE( ${CMAKE_SOURCE_DIR}/ )
robocomp_idsl_to_src(IDSL_SOURCES Camera CommonBehavior CommonHead DifferentialRobot GenericBase OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display )
MESSAGE(STATUS "New files ${IDSL_SOURCES}")



add_library(qlog INTERFACE)
target_include_directories(qlog INTERFACE ${CMAKE_SOURCE_DIR}/classes/ )

# OPENSG
find_package(OpenSceneGraph 2.0.0 REQUIRED osgGA osgDB osgUtil )
add_library(osg INTERFACE IMPORTED)
set_property(TARGET osg PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${OPENSCENEGRAPH_INCLUDE_DIRS})

# Qt
find_package(Qt5 COMPONENTS Widgets OpenGL Core REQUIRED)
add_library(qt5_req INTERFACE IMPORTED)
set_property(TARGET qt5_req PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${Qt5_INCLUDE_DIR})

# ICE
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice IceStorm OPTIONAL_COMPONENTS IceUtil )
#include_directories ()

# Python bindings
#set(Boost_USE_STATIC_LIBS   OFF)
#set(Boost_USE_MULTITHREADED ON)
#if (PYTHON_BINDINGS_SUPPORT)
#	# Add bindings source code file
#	SET(srcs ${srcs} innermodel_python.cpp)
#	find_package(Boost COMPONENTS system thread python REQUIRED)
#	find_package(PythonLibs REQUIRED)
#	MESSAGE(STATUS "im PYTHON BINDINGS: DIRS ${PYTHON_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
#	MESSAGE(STATUS "im PYTHON BINDINGS: LIBS ${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS}")
#	link_directories(${PYTHON_LIBRARIES} ${Boost_LIBRARY_DIRS_DIRS})
#endif (PYTHON_BINDINGS_SUPPORT)


# Specify construction and link process
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(RCIS_SOURCES ${SPECIFIC_SOURCES}
		${IDSL_SOURCES}
		${ICES_SOURCES}
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
		displayI.cpp
		touchsensorI.cpp
		${CMAKE_SOURCE_DIR}/classes/rapplication/rapplication.cpp
		${CMAKE_SOURCE_DIR}/classes/qlog/qlog.cpp
		)
add_executable( rcis ${RCIS_SOURCES})
target_sources(rcis
		PRIVATE
		${RCIS_SOURCES}
		)
target_link_libraries( rcis
		PRIVATE
		qlog
		Qt5::Widgets
		${Ice_LIBRARIES}
		${OPENSCENEGRAPH_LIBRARIES}
		Robocomp::qmat
		Robocomp::innermodel
		Robocomp::osgviewer
		pthread
		)

target_include_directories(rcis
		PRIVATE
		${Ice_INCLUDE_DIRS})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rcis DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
