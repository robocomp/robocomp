//
// Created by juancarlos on 18/10/21.
//

#include <dsr/api/dsr_geometry_queries_api.h>

std::vector<std::pair<std::string, float>> geometry_queries_api::nearest(std::tuple<float, float, float> input, size_t n)
{
    auto [x, y, z] = input;
    return p_nearest(GeomInfo::nearest(GeomInfo::point(x, y, z), n), GeomInfo::point(x, y, z));
}
std::vector<std::pair<std::string, float>> geometry_queries_api::nearest(const std::vector<std::tuple<float, float, float>>& input , size_t n)
{
    GeomInfo::points ls;
    for (auto [x, y, z] : input) boost::geometry::append(ls, GeomInfo::point(x, y, z));
    GeomInfo::point p;
    boost::geometry::centroid(ls, p);
    return p_nearest(GeomInfo::nearest(p, n), p);
}


std::vector<State> geometry_queries_api::status(std::tuple<float, float, float> input, const std::string& geom)
{
    auto [x, y, z] = input;
    return p_status(GeomInfo::point(x, y, z), geom);
}
std::vector<State> geometry_queries_api::status(const std::vector<std::tuple<float, float, float>>& input, const std::string& geom)
{
    GeomInfo::points ls;
    for (auto [x, y, z] : input) boost::geometry::append(ls, GeomInfo::point(x, y, z));
    return p_status(ls, geom);
}
