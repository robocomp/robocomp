find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set (gazebo_robocomp_msgs
    laser_data.proto
    diffdrive_state.proto
    diffdrive_cmd.proto
    motor_goal_velocity.proto
    jointmotor_params.proto
    jointmotor_state.proto
    motor_goal_position.proto
    motor_goal_pos_list.proto
    motor_goal_vel_list.proto
    motor_params_list.proto
    motor_state_list.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${gazebo_robocomp_msgs})

add_library(laser_data SHARED ${PROTO_SRCS})
target_link_libraries(laser_data ${PROTOBUF_LIBRARY})

add_library(diffdrive_cmd SHARED ${PROTO_SRCS})
target_link_libraries(diffdrive_cmd ${PROTOBUF_LIBRARY})

add_library(diffdrive_state SHARED ${PROTO_SRCS})
target_link_libraries(diffdrive_state ${PROTOBUF_LIBRARY})

add_library(motor_goal_velocity SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_velocity ${PROTOBUF_LIBRARY})

add_library(motor_goal_position SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_position ${PROTOBUF_LIBRARY})

add_library(jointmotor_state SHARED ${PROTO_SRCS})
target_link_libraries(jointmotor_state ${PROTOBUF_LIBRARY})

add_library(jointmotor_params SHARED ${PROTO_SRCS})
target_link_libraries(jointmotor_params ${PROTOBUF_LIBRARY})

add_library(motor_goal_pos_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_pos_list ${PROTOBUF_LIBRARY})

add_library(motor_goal_vel_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_goal_vel_list ${PROTOBUF_LIBRARY})

add_library(motor_params_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_params_list ${PROTOBUF_LIBRARY})

add_library(motor_state_list SHARED ${PROTO_SRCS})
target_link_libraries(motor_state_list ${PROTOBUF_LIBRARY})
