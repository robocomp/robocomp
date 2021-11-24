//
// Created by juancarlos on 18/10/21.
//

#ifndef DSR_DSR_GEOMETRY_QUERIES_H
#define DSR_DSR_GEOMETRY_QUERIES_H

#include <memory>
#include "GeometryInfo.h"

enum State {
    within,
    covered_by,
    overlaps,
    intersects,
    contains,
    disjoint,
    over,
    under
};

class geometry_queries_api {

    template<typename T, typename G> inline std::vector<std::pair<std::string, float>> p_nearest(T input, const G& n)
    {
        return m_geom_info->query(input, n);
    }

    template<typename T> inline std::vector<State> p_status(T input, const std::string& geom)
    {
        std::vector<State> state;
        GeomInfo::box bbox = m_geom_info->get_geom(geom);
        if (GeomInfo::within(input, bbox)) state.emplace_back(State::within);
        if (GeomInfo::covered_by(input, bbox)) state.emplace_back(State::covered_by);
        //if (GeomInfo::overlaps(input, bbox)) state.emplace_back(State::overlaps);
        if (GeomInfo::intersects(input, bbox)) state.emplace_back(State::intersects);
        //if (GeomInfo::contains(input, bbox)) state.emplace_back(State::contains);
        if (GeomInfo::disjoint(input, bbox)) state.emplace_back(State::disjoint);
        if (GeomInfo::over(input, bbox)) state.emplace_back(State::over);
        if (GeomInfo::under(input, bbox)) state.emplace_back(State::under);

        return state;
    }

public:
    geometry_queries_api()
        : m_geom_info(TempSingleton<GeomInfo>::get())
    {}

    std::vector<std::pair<std::string, float>> nearest(std::tuple<float, float, float> input, size_t n);
    std::vector<std::pair<std::string, float>> nearest(const std::vector<std::tuple<float, float, float>>& input, size_t n);

    //TODO: Dejar hacer otras consultas?


    std::vector<State> status(std::tuple<float, float, float> input, const std::string& geom);
    std::vector<State> status(const std::vector<std::tuple<float, float, float>>& input, const std::string& geom);

    std::vector<std::tuple<float, float, float>> get_geom_bbox_vertices(const std::string& geom);
    std::pair<std::vector<std::tuple<float, float, float>>, std::vector<unsigned short>> get_geom_vertices_and_indices(const std::string& geom);

private:

    std::shared_ptr<GeomInfo> m_geom_info; // This object is shared with the Qt3D Viewer.
};


#endif //DSR_DSR_GEOMETRY_QUERIES_H
