//
// Created by robolab on 26/5/21.
//

#ifndef DSR_GRAPH_COLORS_H
#define DSR_GRAPH_COLORS_H
#include "node_colors.h"
#include "edge_colors.h"

template<typename Ta>
class GraphColors
{
    std::string default_node_color="blue";
    std::string default_edge_color="black";
public:
    std::string operator[](std::string key) const
    {
        std::map<std::string,std::string> colors;
        std::string default_color = "blue";
        if constexpr(std::is_same_v<Ta, DSR::Edge>)
        {
            colors = edge_colors;
            default_color= default_edge_color;
        }
        else if constexpr(std::is_same_v<Ta, DSR::Node>)
        {
            colors = node_colors;
            default_color = default_node_color;
        }
        if(colors.count(key)>0)
            return colors[key];
        else
            return default_color;
    };
};

#endif //DSR_GRAPH_COLORS_H
