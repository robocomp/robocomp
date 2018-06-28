cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(gazebo_robocomp_pkgs)

FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

include(FindBoost)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11 -lpcl_filters")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
    pkg_check_modules(SDF sdformat)
    pkg_check_modules(OPENCV opencv)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Paging OGRE-Paging)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE-Paging_INCLUDE_DIRS}
    ${OGRE-Terrain_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/../../build/tools/gzserver/gazebo_robocomp_msgs
    ${easyiceconfig_INCLUDE_DIRS}
    ice-interface/include
    slice_cpp/include
) 

link_directories(
    ${GAZEBO_LIBRARY_DIRS} 
    gazebo_robocomp_msgs 
    ${OpenCV_LIBRARIES} 
    ${easyiceconfig_LIBRARY_DIRS} 
    /usr/lib/x86_64-linux-gnu/c++11
    pthread 
    ${ZeroCIce_LIBRARIES} 
)

add_subdirectory(gazebo_robocomp_msgs)
add_subdirectory(gazebo_plugins)
add_subdirectory(slice)
add_subdirectory(slice_cpp)
add_subdirectory(ice-interface)

add_executable(joint_vel src/vel.cc)
target_link_libraries(joint_vel 
    motor_goal_pos_list 
    motor_goal_vel_list 
    motor_params_list 
    motor_state_list 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES} 
    ${SDF_LIBRARIES}
)

add_executable(My_Robot_Vel src/diff_drive_cmd.cc)
target_link_libraries(My_Robot_Vel diffdrive_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(My_Robot_Vel diffdrive_msgs)

add_executable(gazeboserver src/server.cpp)
target_link_libraries(gazeboserver 
    Laser 
    LaserI 
    Ice 
    IceUtil
    JointMotorI
    MotorI
    RGBDI
    IMUI
    DifferentialRobotI
    CameraI
    bumperI
    Motors
    JointMotor
    DifferentialRobot
    bumper
    Camera
    IMU
    RGBD 
    ${ZeroCIce_LIBRARIES} 
    ${easyiceconfig_LIBRARIES} 
    ${GAZEBO_LIBRARIES} 
    ${Boost_LIBRARIES})
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/gazeboserver DESTINATION /opt/robocomp/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

add_executable(client src/client.cpp)
target_link_libraries(client ${GAZEBO_LIBRARIES} Laser ${Boost_LIBRARIES} Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES})

add_executable(keyboardController src/keyboardController.cpp)
target_link_libraries(keyboardController ${GAZEBO_LIBRARIES} diffdrive_msgs ${Boost_LIBRARIES})
add_dependencies(keyboardController diffdrive_msgs)