include_directories(
    include/
)

add_library(gazebo_robocomp_laser SHARED src/gazebo_robocomp_laser.cc include/gazebo_robocomp_laser.hh)
target_link_libraries(gazebo_robocomp_laser Laser_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES} RayPlugin)
add_dependencies(gazebo_robocomp_laser Laser_msgs)

add_library(gazebo_robocomp_IMU SHARED src/gazebo_robocomp_IMU.cc include/gazebo_robocomp_IMU.hh)
target_link_libraries(gazebo_robocomp_IMU ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_joint SHARED src/gazebo_robocomp_joint.cc include/gazebo_robocomp_joint.hh)
target_link_libraries(gazebo_robocomp_joint 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIS} 
    ${SDF_LIBRARIES} 
    motor_goal_velocity 
    motor_goal_position 
    jointMotorState 
    jointMotor_params
)

add_library(gazebo_robocomp_jointmotor SHARED src/gazebo_robocomp_jointmotor.cc include/gazebo_robocomp_jointmotor.hh)
target_link_libraries(gazebo_robocomp_jointmotor 
    motor_goal_pos_list 
    motor_goal_vel_list 
    motor_params_list 
    motor_state_list 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIS} 
    ${SDF_LIBRARIES} 
)

add_library(gazebo_robocomp_DiffDrive SHARED src/gazebo_robocomp_DiffDrive.cc include/gazebo_robocomp_DiffDrive.hh)
target_link_libraries(gazebo_robocomp_DiffDrive diffdrive_msgs diffdrive_state ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(gazebo_robocomp_DiffDrive diffdrive_msgs diffdrive_state)

add_library(gazebo_robocomp_camera SHARED src/gazebo_robocomp_camera.cc include/gazebo_robocomp_camera.hh)
target_link_libraries(gazebo_robocomp_camera 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${SDF_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${OGRE_LIBRARIES} 
    ${OGRE-Paging_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    CameraPlugin
)

add_library(gazebo_robocomp_bumper SHARED src/gazebo_robocomp_bumper.cc include/gazebo_robocomp_bumper.hh)
target_link_libraries(gazebo_robocomp_bumper ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_library(gazebo_robocomp_RGBD SHARED src/gazebo_robocomp_RGBD.cc include/gazebo_robocomp_RGBD.hh)
target_link_libraries(gazebo_robocomp_RGBD 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${SDF_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${OGRE_LIBRARIES} 
    ${OGRE-Paging_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    DepthCameraPlugin
)
