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
        "accessible"sv,
        "graspable"sv,
        "talking"sv,
        "looking-at"sv,
        "sitting"sv,
        "standing"sv,
        "close-to"sv,
        "has"sv,
        "blocked"sv,
        "is_blocking"sv,
        "is_near"sv,
        "front"sv,
        "interacting"sv,
        "interactive"sv,
        "testtype"sv,
        "thinks"sv
);

#endif //DSR_EDGE_TYPE_H
