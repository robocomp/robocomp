include_directories(
    include/
)

file(MAKE_DIRECTORY src)
file(MAKE_DIRECTORY include)

file(RENAME Laser.h include/Laser.h)
file(RENAME bumper.h include/bumper.h)
file(RENAME Camera.h include/Camera.h)
file(RENAME CommonHead.h include/CommonHead.h)
file(RENAME containers.h include/containers.h)
file(RENAME DifferentialRobot.h include/DifferentialRobot.h)
file(RENAME Exceptions.h include/Exceptions.h)
file(RENAME GenericBase.h include/GenericBase.h)
file(RENAME IMU.h include/IMU.h)
file(RENAME Motors.h include/Motors.h)
file(RENAME RGBD.h include/RGBD.h)
file(RENAME JointMotor.h include/JointMotor.h)

file(RENAME Laser.cpp src/Laser.cpp)
file(RENAME bumper.cpp src/bumper.cpp)
file(RENAME Camera.cpp src/Camera.cpp)
file(RENAME CommonHead.cpp src/CommonHead.cpp)
file(RENAME containers.cpp src/containers.cpp)
file(RENAME DifferentialRobot.cpp src/DifferentialRobot.cpp)
file(RENAME Exceptions.cpp src/Exceptions.cpp)
file(RENAME GenericBase.cpp src/GenericBase.cpp)
file(RENAME IMU.cpp src/IMU.cpp)
file(RENAME Motors.cpp src/Motors.cpp)
file(RENAME RGBD.cpp src/RGBD.cpp)
file(RENAME JointMotor.cpp src/JointMotor.cpp)

add_library(GenericBase SHARED src/GenericBase.cpp include/GenericBase.h)
target_link_libraries(GenericBase 
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(Laser SHARED src/Laser.cpp include/Laser.h)
target_link_libraries(Laser
   GenericBase
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(Motors SHARED src/Motors.cpp include/Motors.h)
target_link_libraries(Motors
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(containers SHARED src/containers.cpp include/containers.h)
target_link_libraries(containers
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(Exceptions SHARED src/Exceptions.cpp include/Exceptions.h)
target_link_libraries(Exceptions
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(IMU SHARED src/IMU.cpp include/IMU.h)
target_link_libraries(IMU
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(DifferentialRobot SHARED src/DifferentialRobot.cpp include/DifferentialRobot.h)
target_link_libraries(DifferentialRobot
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(Camera SHARED src/Camera.cpp include/Camera.h)
target_link_libraries(Camera
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(RGBD SHARED src/RGBD.cpp include/RGBD.h)
target_link_libraries(RGBD
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)

add_library(bumper SHARED src/bumper.cpp include/bumper.h)
target_link_libraries(bumper
   ${Boost_LIBRARIES} 
   Ice
   IceUtil 
   pthread
   #/usr/lib/x86_64-linux-gnu/c++11
)


