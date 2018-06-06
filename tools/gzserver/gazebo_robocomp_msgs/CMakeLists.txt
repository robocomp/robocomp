find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set (gazebo_robocomp_msgs
    Laser_msgs.proto
    raysensor.proto
    diffdrive_state.proto
    diffdrive.proto
    vector3d.proto
    quaternion.proto
    pose.proto
    motor_goal_velocity.proto
    jointMotor_params.proto
    jointMotorState.proto
    motor_goal_position.proto
    image.proto
    motor_goal_pos_list.proto
    motor_goal_vel_list.proto
    motor_params_list.proto
    motor_state_list.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${gazebo_robocomp_msgs})

add_library(Laser_msgs SHARED ${PROTO_SRCS})
target_link_libraries(Laser_msgs ${PROTOBUF_LIBRARY})

add_library(diffdrive_msgs SHARED ${PROTO_SRCS})
target_link_libraries(diffdrive_msgs ${PROTOBUF_LIBRARY})

add_library(diffdrive_state SHARED ${PROTO_SRCS})
target_link_libraries(diffdrive_state ${PROTOBUF_LIBRARY})

add_library(motor_goal_velocity SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_velocity ${PROTOBUF_LIBRARY})

add_library(motor_goal_position SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_position ${PROTOBUF_LIBRARY})

add_library(jointMotorState SHARED ${PROTO_SRCS})
target_link_libraries(jointMotorState ${PROTOBUF_LIBRARY})

add_library(jointMotor_params SHARED ${PROTO_SRCS})
target_link_libraries(jointMotor_params ${PROTOBUF_LIBRARY})

add_library(image SHARED ${PROTO_SRCS})
target_link_libraries(image ${PROTOBUF_LIBRARY})

add_library(motor_goal_pos_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_pos_list ${PROTOBUF_LIBRARY})

add_library(motor_goal_vel_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_vel_list ${PROTOBUF_LIBRARY})

add_library(motor_params_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_params_list ${PROTOBUF_LIBRARY})

add_library(motor_state_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_state_list ${PROTOBUF_LIBRARY})
