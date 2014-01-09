set(LIB_NAME innermodel2)
set(LIB_DESC "RoboComp InnerModel library with physics support")
set(LIB_DEPS qmat osgviewer)

if(BULLET_FOUND)
  set(srcs
	  innermodel.cpp
	  innermodelreader.cpp
	  innermodelviewer.cpp
	  joints/BallJoint.cpp
	  joints/GenericJoint.cpp
	  joints/HingeJoint.cpp
	  joints/Joint.cpp
	  joints/PrismaticJoint.cpp
	  nodes/Body.cpp
	  nodes/DifferentialRobot.cpp
	  nodes/Node.cpp
	  primitives/Mesh.cpp
	  primitives/Plane.cpp
	  primitives/PointCloud.cpp
	  primitives/Primitive.cpp
	  sensors/Camera.cpp
	  sensors/IMU.cpp
	  sensors/Laser.cpp
	  sensors/RGBD.cpp
	  sensors/Sensor.cpp
    )

  set(incs
      include/${LIB_NAME}/innermodel.h
    )
      

  RoboComp_LIB_DEPEND(build ${LIB_NAME} DEPS ${LIB_DEPS} OPT_DEPS)
  set(OUT_FILE_NAME ${PROJECT_NAME_LOWER}_${LIB_NAME})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

  INCLUDE( ${QT_USE_FILE} )
  ADD_DEFINITIONS(${QT_DEFINITIONS})

  RoboComp_ADD_LIBRARY(${OUT_FILE_NAME} SHARED ${LIB_NAME} ${srcs} ${incs} )
  RoboComp_ADD_INCLUDES(${LIB_NAME} ${LIB_NAME} ${incs})

  target_link_libraries( ${OUT_FILE_NAME} ${QT_LIBRARIES} -losg robocomp_qmat osgbCollision)
  
endif(BULLET_FOUND)