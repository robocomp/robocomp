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
robocomp_idsl_to_src(IDSL_SOURCES Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display )
MESSAGE(STATUS "New files ${IDSL_SOURCES}")
#ROBOCOMP_IDSL_TO_ICE( Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display)
#ROBOCOMP_ICE_TO_SRC( Camera CommonBehavior CommonHead DifferentialRobot OmniRobot IMU InnerModelManager JointMotor Laser RGBD TouchSensor RCISMousePicker GenericBase Display)
#SET(ROBOCOMP_LIBS ${ROBOCOMP_LIBS} -lrobocomp_qmat )

#include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../../libs/innermodel/include )
#include_directories(${CMAKE_SOURCE_DIR}/libs/innermodel/include)
#include_directories(${CMAKE_SOURCE_DIR}/libs/qmat/include)
#include_directories(${CMAKE_SOURCE_DIR}/libs/osgviewer/include)

add_library(qlog INTERFACE)
target_include_directories(qlog INTERFACE ${CMAKE_SOURCE_DIR}/classes/ )

# OPENSG
find_package(OpenSceneGraph 2.0.0 REQUIRED osgDB osgUtil)
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
include_directories (${Ice_INCLUDE_DIRS})
# Python bindings
set(Boost_USE_STATIC_LIBS   OFF)
set(Boost_USE_MULTITHREADED ON)
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
#SET (EXECUTABLE_OUTPUT_PATH ../bin)
add_executable( rcis  )
target_sources(rcis
		PRIVATE
		${SPECIFIC_SOURCES}
		${IDSL_SOURCES}
#		${ICES_SOURCES}
#		${SOURCES}
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
#		${CMAKE_SOURCE_DIR}/classes/rapplication/rapplication.cpp
#		${CMAKE_SOURCE_DIR}/classes/qlog/qlog.cpp
#		${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS}
		)
target_link_libraries( rcis
		PUBLIC
		qlog
		PRIVATE
		Qt5::Core
		Qt5::Widgets
		${Ice_LIBRARIES}
		${OPENSCENEGRAPH_LIBRARIES}
		qmat
		innermodel
		)
target_include_directories(rcis
		PRIVATE
		${CMAKE_CURRENT_BINARY_DIR}
		.)
#		${CMAKE_CURRENT_BINARY_DIR}
#		${PYTHON_INCLUDE_DIRS}
#		${Boost_INCLUDE_DIRS}
#		${CMAKE_SOURCE_DIR}/libs/innermodel/include
#		${CMAKE_SOURCE_DIR}/libs/osgviewer/include
#		${CMAKE_SOURCE_DIR}/libs/qmat/include
#		)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/rcis DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
