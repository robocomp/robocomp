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
        { "in", "brown"},
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
        { "has", "yellow"},
        { "blocked", "blue"},
        { "is_blocking", "blue"},
        { "is_near", "blue"},
        { "front", "blue"},
        { "interacting", "orange"},
        { "interactive", "blue"},
        { "thinks", "blue"},
        { "goto_action", "green"},
        { "attention_action", "magenta"}
};

#endif //DSR_EDGE_COLORS_H
