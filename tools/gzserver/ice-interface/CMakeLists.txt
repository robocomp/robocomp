include_directories(
    include/
)

add_library(bumperI SHARED src/bumperI.cpp include/bumperI.h)
target_link_libraries(bumperI bumper Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(CameraI SHARED src/CameraI.cpp include/CameraI.h)
target_link_libraries(CameraI Camera Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(RGBDI SHARED src/RGBDI.cpp include/RGBDI.h)
target_link_libraries(RGBDI RGBD Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(LaserI SHARED src/LaserI.cpp include/LaserI.h)
target_link_libraries(LaserI Laser Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(MotorI SHARED src/MotorI.cpp include/MotorI.h)
target_link_libraries(MotorI Motors Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(IMUI SHARED src/IMUI.cpp include/IMUI.h)
target_link_libraries(IMUI IMU Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(DifferentialRobotI SHARED src/DifferentialRobotI.cpp include/DifferentialRobotI.h)
target_link_libraries(DifferentialRobotI DifferentialRobot diffdrive_msgs diffdrive_state Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(DifferentialRobotI diffdrive_msgs diffdrive_state)

add_library(JointMotorI SHARED src/JointMotorI.cpp include/JointMotorI.h)
target_link_libraries(JointMotorI JointMotor motor_goal_pos_list motor_goal_vel_list motor_params_list motor_state_list Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
