//
// Created by juancarlos on 18/8/20.
//

#ifndef DSR_NODE_TYPE_H
#define DSR_NODE_TYPE_H

#include <string_view>
#include <array>
#include <experimental/array>

using namespace std::literals;

constexpr auto node_type_names = std::experimental::make_array(
        "world"sv,
        "room"sv,
        "differentialrobot"sv,
        "omnirobot"sv,
        "robot"sv,
        "battery"sv,
        "pose"sv,
        "laser"sv,
        "camera"sv,
        "imu"sv,
        "slam_device"sv,
        "object"sv,
        "affordance_space"sv,
        "person"sv,
        "personal_space"sv,
        "plane"sv,
        "box"sv,
        "cylinder"sv,
        "ball"sv,
        "mesh"sv,
        "face"sv,
        "body"sv,
        "chest"sv,
        "nose"sv,
        "left_eye"sv,
        "right_eye"sv,
        "left_ear"sv,
        "right_ear"sv,
        "left_arm"sv,
        "right_arm"sv,
        "left_shoulder"sv,
        "right_shoulder"sv,
        "left_elbow"sv,
        "right_elbow"sv,
        "left_wrist"sv,
        "right_wrist"sv,
        "left_hip"sv,
        "right_hip"sv,
        "left_leg"sv,
        "right_leg"sv,
        "left_knee"sv,
        "right_knee"sv,
        "mug"sv,
        "noodles"sv,
        "table"sv,
        "chair"sv,
        "shelve"sv,
        "dish"sv,
        "spoon"sv,
        "testtype"sv
);

#endif //DSR_NODE_TYPE_H
