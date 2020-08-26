//
// Created by juancarlos on 18/8/20.
//

#ifndef DSR_EDGE_TYPE_H
#define DSR_EDGE_TYPE_H

#include <string_view>
#include <array>
#include <experimental/array>

using namespace std::literals;

constexpr auto edge_type_names = std::experimental::make_array(
        "RT"sv,
        "reachable"sv,
        "in"sv,
        "knows"sv,
        "transitable"sv,
        "graspable"sv,
        "talking"sv,
        "looking-at"sv,
        "sitting"sv,
        "standing"sv,
        "close-to"sv,
        "has"sv,
        "block"sv,
        "softBlock"sv,
        "affordanceBlock"sv,
        "Interacting"sv,
        "Interactive"sv,
        "testtype"sv
);

#endif //DSR_EDGE_TYPE_H
