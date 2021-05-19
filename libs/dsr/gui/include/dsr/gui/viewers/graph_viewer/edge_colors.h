//
// Created by robolab on 19/5/21.
//

#ifndef DSR_EDGE_COLORS_H
#define DSR_EDGE_COLORS_H

#include <map>
#include <string>

static const std::map<std::string, std::string> edge_colors = {
        { "RT", "black"},
        { "reachable", "blue"},
        { "in", "red"},
        { "knows", "blue"},
        { "transitable", "blue"},
        { "visible", "blue"},
        { "accessible", "blue"},
        { "graspable", "blue"},
        { "talking", "blue"},
        { "looking_at", "blue"},
        { "sitting", "blue"},
        { "standing", "blue"},
        { "close_to", "blue"},
        { "has", "orange"},
        { "blocked", "blue"},
        { "is_blocking", "blue"},
        { "is_near", "blue"},
        { "front", "blue"},
        { "interacting", "red"},
        { "interactive", "blue"},
        { "thinks", "blue"},
        { "goto_action", "blue"},
        { "attention_action", "blue"}
};

#endif //DSR_EDGE_COLORS_H
