//
// Created by juancarlos on 18/8/20.
//

#ifndef DSR_NODE_TYPE_H
#define DSR_NODE_TYPE_H

#include <string_view>
#include <array>
#include <experimental/array>
#include "type_checker.h"
using namespace std::literals;

template<const std::string_view& n>
struct NodeType {
    static constexpr bool node_type = true;
    static constexpr std::string_view attr_name = std::string_view(n);
};


#define REGISTER_NODE_TYPE(x) \
                                static constexpr auto    x ##_type_str = std::string_view(#x ); \
                                [[maybe_unused]] inline bool x ##__b =  node_types::register_type( x##_type_str);     \
                                using x##_node_type = NodeType< x##_type_str >;                      \
                                \

//typedef NodeType< x##_type_str> x##_type


inline std::unordered_set<std::string_view> node_types::set_type_;


REGISTER_NODE_TYPE(world)
REGISTER_NODE_TYPE(transform)
REGISTER_NODE_TYPE(room)
REGISTER_NODE_TYPE(differentialrobot)
REGISTER_NODE_TYPE(omnirobot)
REGISTER_NODE_TYPE(robot)
REGISTER_NODE_TYPE(battery)
REGISTER_NODE_TYPE(pose)
REGISTER_NODE_TYPE(laser)
REGISTER_NODE_TYPE(camera)
REGISTER_NODE_TYPE(imu)
REGISTER_NODE_TYPE(slam_device)
REGISTER_NODE_TYPE(object)
REGISTER_NODE_TYPE(affordance_space)
REGISTER_NODE_TYPE(person)
REGISTER_NODE_TYPE(personal_space)
REGISTER_NODE_TYPE(plane)
REGISTER_NODE_TYPE(box)
REGISTER_NODE_TYPE(cylinder)
REGISTER_NODE_TYPE(ball)
REGISTER_NODE_TYPE(mesh)
REGISTER_NODE_TYPE(face)
REGISTER_NODE_TYPE(body)
REGISTER_NODE_TYPE(chest)
REGISTER_NODE_TYPE(nose)
REGISTER_NODE_TYPE(left_eye)
REGISTER_NODE_TYPE(right_eye)
REGISTER_NODE_TYPE(left_ear)
REGISTER_NODE_TYPE(right_ear)
REGISTER_NODE_TYPE(left_arm)
REGISTER_NODE_TYPE(right_arm)
REGISTER_NODE_TYPE(left_shoulder)
REGISTER_NODE_TYPE(right_shoulder)
REGISTER_NODE_TYPE(left_elbow)
REGISTER_NODE_TYPE(right_elbow)
REGISTER_NODE_TYPE(left_wrist)
REGISTER_NODE_TYPE(right_wrist)
REGISTER_NODE_TYPE(left_hand)
REGISTER_NODE_TYPE(right_hand)
REGISTER_NODE_TYPE(left_hip)
REGISTER_NODE_TYPE(right_hip)
REGISTER_NODE_TYPE(left_leg)
REGISTER_NODE_TYPE(right_leg)
REGISTER_NODE_TYPE(left_knee)
REGISTER_NODE_TYPE(right_knee)
REGISTER_NODE_TYPE(mug)
REGISTER_NODE_TYPE(cup)
REGISTER_NODE_TYPE(noodles)
REGISTER_NODE_TYPE(table)
REGISTER_NODE_TYPE(chair)
REGISTER_NODE_TYPE(shelve)
REGISTER_NODE_TYPE(dish)
REGISTER_NODE_TYPE(spoon)
REGISTER_NODE_TYPE(testtype)
REGISTER_NODE_TYPE(glass)
REGISTER_NODE_TYPE(path_to_target)
REGISTER_NODE_TYPE(intention)
REGISTER_NODE_TYPE(rgbd)
REGISTER_NODE_TYPE(pan_tilt)

//melex-rodao types
REGISTER_NODE_TYPE(road)
REGISTER_NODE_TYPE(building)
REGISTER_NODE_TYPE(vehicle)
REGISTER_NODE_TYPE(gps)
REGISTER_NODE_TYPE(grid)


#endif //DSR_NODE_TYPE_H
