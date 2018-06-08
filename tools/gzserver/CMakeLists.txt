cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(gazebo_robocomp_pkgs)

FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
# find_package(Ice REQUIRED COMPONENTS
    # Ice 
    # IceUtil
    # Glacier2 
    # IceBox 
    # IceGrid 
    # IcePatch2 
    # IceStorm
# )
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
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/../../build/tools/gzserver/gazebo_robocomp_msgs
    ${easyiceconfig_INCLUDE_DIRS}
    ice-interface/include
    slice_cpp/include
) 

link_directories(
    ${GAZEBO_LIBRARY_DIRS} 
    gazebo_robocomp_msgs 
    ${OpenCV_LIBRARIES} 
    ${PCL_LIBRARY_DIRS} 
    ${easyiceconfig_LIBRARY_DIRS} 
    # IceUtil 
    /usr/lib/x86_64-linux-gnu/c++11
    # Ice
    pthread 
    ${ZeroCIce_LIBRARIES} 
)

add_definitions(${PCL_DEFINITIONS})
add_subdirectory(gazebo_robocomp_msgs)
add_subdirectory(gazebo_plugins)
add_subdirectory(slice)
add_subdirectory(slice_cpp)
add_subdirectory(ice-interface)

add_executable(joint_vel src/vel.cc)
target_link_libraries(joint_vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_executable(My_Robot_Vel src/diff_drive_cmd.cc)
target_link_libraries(My_Robot_Vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})

add_executable(server src/server.cpp)
target_link_libraries(server Laser LaserI Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(client src/client.cpp)
target_link_libraries(client ${GAZEBO_LIBRARIES} Laser ${Boost_LIBRARIES} Ice IceUtil ${ZeroCIce_LIBRARIES} ${easyiceconfig_LIBRARIES})
