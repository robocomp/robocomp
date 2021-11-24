//
// Created by juancarlos on 1/10/21.
//

#ifndef QT3D_PRUEBAS_GEOMETRYINFO_H
#define QT3D_PRUEBAS_GEOMETRYINFO_H


#include <vector>
#include <mutex>
#include <shared_mutex>
#include <iostream>

#include <Qt3DCore/QEntity>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

template<typename Type>
class TempSingleton {
public:

    static std::shared_ptr<Type> get()
    {
        std::unique_lock<std::mutex> lock(mtx);
        auto shared = m_object.lock();
        if (!shared)
        {
            shared = std::shared_ptr<Type>(new Type());
            m_object = shared;
        }
        lock.unlock();
        return shared;
    }

private:
    static inline std::mutex mtx;
    static inline std::weak_ptr<Type> m_object;
};


class GeomInfo : public QObject {
    Q_OBJECT

private:

    GeomInfo() = default;

public:
    friend std::shared_ptr<GeomInfo> TempSingleton<GeomInfo>::get();

    GeomInfo(const GeomInfo&) = delete;

    typedef boost::geometry::model::point<float, 3, boost::geometry::cs::cartesian> point;
    typedef boost::geometry::model::polygon<point, false> poly;
    typedef boost::geometry::model::box<point> box;
    typedef boost::geometry::model::linestring<point> points;
    typedef std::pair<box, unsigned> value;

    void addQtGeom(Qt3DCore::QEntity* eptr, std::vector<std::tuple<float, float, float>> && positions, std::vector<unsigned short> && indices)
    {
        poly p;
        for (auto [x, y, z] : positions)
        {
            p.outer().push_back(point(x, y, z));
        }

        box b = boost::geometry::return_envelope<box>(p);
        std::unique_lock<std::shared_mutex> lock(mtx);
        rtree.insert(std::make_pair(b, idx_geom));
        rtree_.emplace(idx_geom, b);
        std::string name = eptr->objectName().toStdString();
        rtree_name_idx.emplace(name, idx_geom);
        rtree_idx.emplace(idx_geom++, name);
        geometries.emplace(eptr, std::pair(std::move(positions), std::move(indices)));
        e_name_map.emplace(name, eptr);
        lock.unlock();
        emit geometry_added(eptr);
    }

    std::optional<std::pair<std::vector<std::tuple<float, float, float>>, std::vector<unsigned short>>> getQtGeom(Qt3DCore::QEntity* eptr)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        if (auto it = geometries.find(eptr); it != geometries.end())
            return it->second;
        else return {};
    }


    std::optional<std::pair<std::vector<std::tuple<float, float, float>>, std::vector<unsigned short>>> getQtGeom(const std::string& name)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        if (auto it1 = e_name_map.find(name); it1 != e_name_map.end() )
            if (auto it = geometries.find(it1->second); it != geometries.end())
                return it->second;

        return {};
    }

    std::vector<std::tuple<float, float, float>> getGeomBbox(Qt3DCore::QEntity* eptr)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        auto box = rtree_[rtree_name_idx[eptr->objectName().toStdString()]];
        lock.unlock();
        auto max = box.max_corner();
        auto min = box.min_corner();
        auto v1 = std::tuple<float, float, float>(min.get<0>(), min.get<1>(), max.get<2>());
        auto v2 = std::tuple<float, float, float>(max.get<0>(), min.get<1>(), min.get<2>());
        auto v3 = std::tuple<float, float, float>(min.get<0>(), max.get<1>(), min.get<2>());
        auto v4 = std::tuple<float, float, float>(max.get<0>(), max.get<1>(), min.get<2>());
        auto v5 = std::tuple<float, float, float>(min.get<0>(), max.get<1>(), max.get<2>());
        auto v6 = std::tuple<float, float, float>(max.get<0>(), min.get<1>(), max.get<2>());
        return std::vector{ std::tuple<float, float, float>(min.get<0>(), min.get<1>(), min.get<2>()),
                            std::tuple<float, float, float>(max.get<0>(), max.get<1>(), max.get<2>()),
                            v1, v2, v3, v4, v5, v6};

    }

    std::vector<std::tuple<float, float, float>> getGeomBbox(const std::string& name)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        auto box = rtree_[rtree_name_idx[name]];
        lock.unlock();
        auto max = box.max_corner();
        auto min = box.min_corner();
        auto v1 = std::tuple<float, float, float>(min.get<0>(), min.get<1>(), max.get<2>());
        auto v2 = std::tuple<float, float, float>(max.get<0>(), min.get<1>(), min.get<2>());
        auto v3 = std::tuple<float, float, float>(min.get<0>(), max.get<1>(), min.get<2>());
        auto v4 = std::tuple<float, float, float>(max.get<0>(), max.get<1>(), min.get<2>());
        auto v5 = std::tuple<float, float, float>(min.get<0>(), max.get<1>(), max.get<2>());
        auto v6 = std::tuple<float, float, float>(max.get<0>(), min.get<1>(), max.get<2>());
        return std::vector{ std::tuple<float, float, float>(min.get<0>(), min.get<1>(), min.get<2>()),
                            std::tuple<float, float, float>(max.get<0>(), max.get<1>(), max.get<2>()),
                            v1, v2, v3, v4, v5, v6};

    }
    box get_geom(const std::string& name)
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        return rtree_[rtree_name_idx[name]];
    }



    /*template<typename Geometry> inline static auto contains(const Geometry& g, const box& g2)
    {

        return boost::geometry::within(g2, g);
    }*/

    template<typename Geometry> inline static  auto covered_by(const Geometry& g, const box& g2)
    {
        if constexpr(std::is_same_v<Geometry, point>)
            return boost::geometry::covered_by(g, g2);
        else if constexpr(std::is_same_v<Geometry, points>) {
            bool res = true;
            for (point l : g) {
                res &= boost::geometry::covered_by(l, g2);
                if (!res) return false;
            }
            return res;
        } else return false;
    }

    template<typename Geometry> inline static  auto disjoint(const Geometry& g, const box& g2)
    {
        return boost::geometry::disjoint(g, g2);
    }

    template<typename Geometry> inline static  auto intersects(const Geometry& g, const box& g2)
    {
        return boost::geometry::intersects(g, g2);
    }

    /*template<typename Geometry, typename Geometry2> inline static  auto overlaps(const Geometry& g, const Geometry2& g2)
    {
        return boost::geometry::overlaps(g, g2);
    }*/

    template<typename Geometry> inline static  auto within(const Geometry& g, const box& g2)
    {

        if constexpr(std::is_same_v<Geometry, point>)
            return boost::geometry::within(g, g2);
        else if constexpr(std::is_same_v<Geometry, points>) {
            bool res = true;
            for (point l : g) {
                res &= boost::geometry::within(l, g2);
                if (!res) return false;
            }
            return res;
        } else return false;
    }

    template<typename Geometry> inline static  auto nearest(Geometry const& g, unsigned k)
    {
        return boost::geometry::index::nearest(g, k);
    }

    template<typename Geometry> inline static  auto over(const Geometry& g, const box& g2)
    {
        point p;
        point p2 = g2.max_corner();


        if constexpr(std::is_same_v<Geometry, point>)
        {
            p = g;
        } else {
            p = *std::max_element(g.begin(), g.end(), [&](const point& a, const point &b) { return a.get<2>() < b.get<2>(); });
        }

        box b2 (g2.min_corner(), point(p2.get<0>(),  p2.get<1>(), (p.get<2>() > p2.get<2>()) ? p.get<2>() : p2.get<2>()));

        return boost::geometry::intersects(g, b2) && p.get<2>() >= p2.get<2>();
    }

    template<typename Geometry> inline static  auto under(const Geometry& g, const box& g2)
    {
        point p;
        point p2 = g2.min_corner();


        if constexpr(std::is_same_v<Geometry, point>)
        {
            p = g;
        } else {
            p = *std::min_element(g.begin(), g.end(), [&](const point& a, const point &b) { return a.get<1>() < b.get<1>(); });
        }

        box b2 (point(p2 .get<0>(), p2.get<1>(), (p.get<2>() < p2.get<2>()) ? p.get<2>() : p2.get<2>() ), g2.max_corner());


        return boost::geometry::intersects(g, b2) && p.get<2>() <= p2.get<2>();
    }

    template<typename Predicate, typename Geometry>
    std::vector<std::pair<std::string, float>> query(Predicate pred, const Geometry& geom)
    {

        std::vector<value> result_n;
        std::shared_lock<std::shared_mutex> lock(mtx);
        rtree.query(pred , std::back_inserter(result_n));
        std::vector<std::pair<std::string, float>> ret;
        ret.reserve(result_n.size());
        for (auto &val : result_n) {
            ret.emplace_back(std::pair(rtree_idx.at(std::get<1>(val)),
                                       boost::geometry::distance(geom, rtree.indexable_get()(val))));
        }
        lock.unlock();
        return ret;
    }


    Q_SIGNALS:

    void geometry_added(Qt3DCore::QEntity* eptr);

private:
    std::shared_mutex mtx;

    std::unordered_map<Qt3DCore::QEntity*, std::pair<std::vector<std::tuple<float, float, float>>, std::vector<unsigned short>>> geometries;
    boost::geometry::index::rtree<value, boost::geometry::index::rstar<16, 4>> rtree;
    std::unordered_map<size_t, box> rtree_;
    std::unordered_map<size_t, std::string> rtree_idx;
    std::unordered_map<std::string, size_t> rtree_name_idx;
    std::unordered_map<std::string, Qt3DCore::QEntity*> e_name_map;

    size_t idx_geom = 0;

};


#endif //QT3D_PRUEBAS_GEOMETRYINFO_H
