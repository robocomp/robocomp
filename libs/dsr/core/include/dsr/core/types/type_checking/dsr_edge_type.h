//
// Created by juancarlos on 18/8/20.
//

#ifndef DSR_EDGE_TYPE_H
#define DSR_EDGE_TYPE_H

#include <string_view>
#include <array>
#include <experimental/array>
#include "type_checker.h"
using namespace std::literals;



template<const std::string_view& n>
struct EdgeType {
    static constexpr bool edge_type = true;
    static constexpr std::string_view attr_name = std::string_view(n);
};


#define REGISTER_EDGE_TYPE(x) \
                                static constexpr auto    x ##_edge_type_str = std::string_view(#x ); \
                                [[maybe_unused]] inline bool x ##__b =  edge_types::register_type( x##_edge_type_str);     \
                                using x##_edge_type = EdgeType< x##_edge_type_str >;                      \
                                \


inline std::unordered_set<std::string_view> edge_types::set_type_;

REGISTER_EDGE_TYPE(RT)
REGISTER_EDGE_TYPE(reachable)
REGISTER_EDGE_TYPE(in)
REGISTER_EDGE_TYPE(knows)
REGISTER_EDGE_TYPE(transitable)
REGISTER_EDGE_TYPE(visible)
REGISTER_EDGE_TYPE(accessible)
REGISTER_EDGE_TYPE(graspable)
REGISTER_EDGE_TYPE(talking)
REGISTER_EDGE_TYPE(looking_at)
REGISTER_EDGE_TYPE(sitting)
REGISTER_EDGE_TYPE(standing)
REGISTER_EDGE_TYPE(close_to)
REGISTER_EDGE_TYPE(has)
REGISTER_EDGE_TYPE(blocked)
REGISTER_EDGE_TYPE(is_blocking)
REGISTER_EDGE_TYPE(is_near)
REGISTER_EDGE_TYPE(front)
REGISTER_EDGE_TYPE(interacting)
REGISTER_EDGE_TYPE(interactive)
REGISTER_EDGE_TYPE(thinks)
REGISTER_EDGE_TYPE(goto_action)
REGISTER_EDGE_TYPE(attention_action)
REGISTER_EDGE_TYPE(testtype_e)

#endif //DSR_EDGE_TYPE_H