set(LIB_NAME innermodel)
set(LIB_DESC "RoboComp InnerModel library")
set(LIB_DEPS qmat osgviewer)

set(srcs
	im1/innermodel.cpp
	im1/innermodelreader.cpp
	im1/innermodelviewer.cpp
   )

set(incs
    im1/include/${LIB_NAME}/innermodel.h
    im1/include/${LIB_NAME}/innermodelreader.h
    im1/include/${LIB_NAME}/innermodelviewer.h
   )

# Bullet
if( BUILD_PHYSICS )
	find_package( Bullet REQUIRED )
	add_deffinitions( -DUSE_BULLET )
	include_directories( ${BULLET_INCLUDE_DIR} )
	set( srcs ${srcs}
		im2/innermodel.cpp
		im2/joints/BallJoint.cpp
		im2/joints/GenericJoint.cpp
		im2/joints/HingeJoint.cpp
		im2/joints/Joint.cpp
		im2/joints/PrismaticJoint.cpp
		im2/nodes/Body.cpp
		im2/nodes/DifferentialRobot.cpp
		im2/nodes/Node.cpp
		im2/primitives/Mesh.cpp
		im2/primitives/Plane.cpp
		im2/primitives/PointCloud.cpp
		im2/primitives/Primitive.cpp
		im2/sensors/Camera.cpp
		im2/sensors/IMU.cpp
		im2/sensors/Laser.cpp
		im2/sensors/RGBD.cpp
		im2/sensors/Sensor.cpp
	)
	set( PHISICS_LIBS ${BULLET_LIBRARIES} osgbCollision )
endif()
    
RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/im1/include ${CMAKE_CURRENT_SOURCE_DIR}/im2/include)

INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS(${QT_DEFINITIONS})

RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${incs} )
RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})

target_link_libraries( ${OUT_FILE_NAME} ${QT_LIBRARIES} -losg robocomp_qmat ${PHISICS_LIBS})

