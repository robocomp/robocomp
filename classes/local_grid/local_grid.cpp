#include "local_grid.h"
#include "qgraphicscellitem.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/slice.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <cppitertools/filterfalse.hpp>
#include <ranges>

void Local_Grid::initialize(  Ranges angle_dim_,
                              Ranges radius_dim_,
                              QGraphicsScene *scene_)
{
    angle_dim = angle_dim_;
    radius_dim = radius_dim_;
    dim = QRectF(angle_dim.init, radius_dim.init, angle_dim.end-angle_dim.init, radius_dim.end-radius_dim.init);
    static std::vector<QGraphicsItem*> lines, circles;
    scene = scene_;
    qInfo() << __FILE__ << __FUNCTION__ <<  "Polar world angle dimension: " << angle_dim.init << angle_dim.end << angle_dim.step;
    qInfo() << __FILE__ << __FUNCTION__ <<  "Polar world radius dimension: " << radius_dim.init << radius_dim.end << radius_dim.step;
    qInfo() << __FILE__ << __FUNCTION__ <<  "dim " << dim;

    // clean existing map
    for (const auto &[key, value]: fmap)
        scene->removeItem(value.tile);
    fmap.clear();

    int id = 0;
    for (int ang = angle_dim.init; ang < angle_dim.end; ang += angle_dim.step)
        for (int rad = radius_dim.init; rad < radius_dim.end; rad += radius_dim.step)
        {
            T aux;
            aux.id = id++;
            aux.free = true;
            aux.visited = false;
            aux.cost = 1.0;
            QGraphicsCellItem *tile = new QGraphicsCellItem(qDegreesToRadians((float)ang), rad, qDegreesToRadians((float)angle_dim.step), radius_dim.step);
            tile->setPos(rad*sin(qDegreesToRadians((float)ang)), rad*cos(qDegreesToRadians((float)ang)));
            tile->setRotation(-ang);
            scene->addItem(tile);
            aux.tile = tile;
            fmap.insert(std::make_pair(Key(ang, rad), aux));
        }

    //auto kv = std::views::keys(fmap);
    //keys_vector.assign(kv.begin(), kv.end());
    //std::ranges::sort(keys_vector, [](auto a, auto b){return (a.ang*a.ang)+(a.rad*a.rad) < (b.ang*b.ang)+(b.rad*b.rad);});

    // draw outlines
   for(auto &e : lines)
       scene->removeItem(e);
   for(auto &c : circles)
       scene->removeItem(c);
   QColor outline_color("lightblue");
   for (float ang = 0; ang < angle_dim.end; ang += angle_dim.step)
       lines.push_back(scene->addLine(0, 0, radius_dim.end*sin(qDegreesToRadians((float)ang)), radius_dim.end*cos(qDegreesToRadians((float)ang)), QPen(outline_color, 30)));
   for (float rad = radius_dim.init; rad < radius_dim.end; rad += radius_dim.step)
   {
       auto s = scene->addEllipse(0, 0, rad*2, rad*2, QPen(outline_color, 20));
       s->setPos(-rad, -rad);
       circles.push_back(s);
   }

   // create cost matrix
   costs = cv::Mat::ones(angle_dim.size(), radius_dim.size(), CV_8UC1);   // rows, cols
}
void Local_Grid::insert(const Key &key, const T &value)
{
    fmap.insert(std::make_pair(key, value));
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(int ang, int rad)
{
    //qInfo() << __FUNCTION__ << ((ang<angle_dim.init) or ang>=angle_dim.end or rad<radius_dim.init or rad>=radius_dim.end);
    if(ang<angle_dim.init or ang>=angle_dim.end or rad<radius_dim.init or rad>=radius_dim.end)
        return std::forward_as_tuple(false, T());
    else
    {
        Key key = pointToKey(ang, rad);
        //qInfo() << __FUNCTION__ << key.ang << key.rad;
        try
        { return std::forward_as_tuple(true, fmap.at(key)); }
        catch (const std::exception &e)
        {
            //qWarning() << __FUNCTION__ << " No key found in grid: (" << key.ang << key.rad << ")";
            return std::forward_as_tuple(false, T());
        }
    }
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(const Key &k)
{
    if(k.ang<angle_dim.init or k.ang>=angle_dim.end or k.rad<radius_dim.init or k.rad>=radius_dim.end)
        return std::forward_as_tuple(false, T());
    else
      try
      {
          return std::forward_as_tuple(true, fmap.at(pointToKey(k.ang, k.rad)));
      }
      catch(const std::exception &e)
      {
          qWarning() << __FUNCTION__ << " No key found in grid: (" << k.ang << k.rad << ")";
          return std::forward_as_tuple(false, T());
      }
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(const Eigen::Vector2f &p)  //polar coordinates
{
    return getCell(p.x(), p.y());
}
Local_Grid::Key Local_Grid::pointToKey(int ang, int rad) const
{
    //we need to get the closest key
    //auto const it = std::ranges::lower_bound(keys_vector, Key(ang, rad), [](auto a, auto b){ return (a.ang*a.ang)+(a.rad*a.rad) < (b.ang*b.ang)+(b.rad*b.rad);});
    //if (it == keys_vector.end()) { return Key(); }
    //return *it;

    int ka = rint((ang - angle_dim.init) / angle_dim.step);
    int kr = rint((rad - radius_dim.init) / radius_dim.step);
    return Key(angle_dim.init + ka * angle_dim.step, radius_dim.init + kr * radius_dim.step);
};
Local_Grid::Key Local_Grid::pointToKey(const QPointF &p) const
{
    int ka = rint((p.x() - dim.left()) / TILE_SIZE);
    int kr = rint((p.y() - dim.top()) / TILE_SIZE);
    return Key(dim.left() + ka * TILE_SIZE, dim.top() + kr * TILE_SIZE);
};
Local_Grid::Key Local_Grid::pointToKey(const Eigen::Vector2f &p) const
{
    int ka = rint((p.x() - dim.left()) / TILE_SIZE);
    int kr = rint((p.y() - dim.top()) / TILE_SIZE);
    return Key(dim.left() + ka * TILE_SIZE, dim.top() + kr * TILE_SIZE);
};

//////////////////////////////// STATUS //////////////////////////////////////////
bool Local_Grid::is_occupied(const Eigen::Vector2f &p)
{
    const auto &[success, v] = getCell(p.x(),p.y());
    if(success)
        return not v.free;
    else
        return true;  // non existing cells are returned as occupied
}
void Local_Grid::set_free(float ang, float rad)
{
    auto &&[success, v] = getCell(ang, rad);
    if(success)
    {
        v.free = true;
        if (v.tile != nullptr)
            v.tile->setFreeColor();
    }
}
void Local_Grid::set_free(const QPointF &p)
{
    set_free(p.x(), p.y());
}

void Local_Grid::setOccupied(float ang, float rad)
{
    auto &&[success, v] = getCell(ang, rad);
    if(success)
    {
        v.free = false;
        if(v.tile != nullptr)
            v.tile->setOccupiedColor(0);
    }
}
void Local_Grid::setOccupied(const QPointF &p)
{
    setOccupied(p.x(), p.y());
}

void Local_Grid::add_miss(const Eigen::Vector2f &p)
{
    auto pd = radians_to_degrees(p.x());
    auto &&[success, v] = getCell(pd, p.y());
    if(success)
    {
        v.misses++;
        if((float)v.hits/(v.hits+v.misses) < params.occupancy_threshold)
        {
            v.free = true;
            if(v.tile != nullptr)
                v.tile->setFreeColor();
        }
        v.misses = std::clamp(v.misses, 0.f, 20.f);
        this->updated++;
    }
}
void Local_Grid::add_hit(const Eigen::Vector2f &p)
{
    auto pd = radians_to_degrees(p.x());
    auto &&[success, v] = getCell(pd, p.y());
    if(success)
    {
        v.hits++;
        if((float)v.hits/(v.hits+v.misses) >= params.occupancy_threshold)
        {
            v.free = false;
            if(v.tile != nullptr)
                v.tile->setOccupiedColor(0);
        }
        v.hits = std::clamp(v.hits, 0.f, 20.f);
        this->updated++;
    }
}
void Local_Grid::setCost(const Key &k,float cost)
{
    auto &&[success, v] = getCell(k);
    if(success)
        v.cost = cost;
}
float Local_Grid::get_cost(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell(p.x(), p.y());
    if(success)
        return v.cost;
    else
        return -1;
}
void Local_Grid::set_all_costs(float value)
{
    for(auto &[key, cell] : fmap)
        cell.cost = value;
}
int Local_Grid::count_total() const
{
    return fmap.size();
}
void Local_Grid::set_all_to_free()
{
    for(auto &[k,v] : fmap)
        set_free(k.ang, k.rad);
}
void Local_Grid::markAreaInGridAs(const QPolygonF &poly, bool free)
{
//    const qreal step = TILE_SIZE / 4;
//    QRectF box = poly.boundingRect();
//    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
//        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
//        {
//            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
//            {
//                if (free)
//                    setFree(pointToKey(x, y));
//                else
//                    setOccupied(pointToKey(x, y));
//            }
//        }
}
void Local_Grid::modifyCostInGrid(const QPolygonF &poly, float cost)
{
//    const qreal step = TILE_SIZE / 4.f;
//    QRectF box = poly.boundingRect();
//    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
//        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
//            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
//                setCost(pointToKey(x, y), cost);
}

////////////////////////////////////// PATH //////////////////////////////////////////////////////////////
std::list<QPointF> Local_Grid::computePath(const QPointF &source_, const QPointF &target_)
{
//    //qInfo() << __FUNCTION__  << " from nose pos: " << source_ << " to " << target_ ;
//    Key source = pointToKey(source_.x(), source_.y());
//    Key target = pointToKey(target_.x(), target_.y());
//
//    // Admission rules
//    if (not dim.contains(target_))
//    {
//        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim << " Returning empty path";
//        return {};
//    }
//    if (not dim.contains(source_))
//    {
//        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
//        return {};
//    }
//    if (source == target)
//    {
//        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
//        return {};
//    }
//    //source in a non-free cell (red cell)
//    if(neighboors_8(source_).empty())
//    {
//        qInfo() << __FUNCTION__ << "Source on an occupied cell: " << source.x << ", " << source.z << "Returning empty path";
//        std::optional<QPointF> new_source = closest_free(source_);
//        source = pointToKey(new_source->x(), new_source->y());
//    }
//    const auto &[success, val] = getCell(source);
//    if(not success)
//    {
////        qWarning() << "Could not find source position in Local_Grid. Returning empty path";
//        return std::list<QPointF>();
//    }
//
//    // vector de distancias inicializado a UINT_MAX
//    std::vector<uint32_t> min_distance(fmap.size(), std::numeric_limits<uint32_t>::max());
//    // initialize source position to 0
//    min_distance[val.id] = 0;
//    // vector de pares<std::uint32_t, Key> initialized to (-1, Key())
//    std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
//    // lambda to compare two vertices: a < b if a.id<b.id or
//    auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y){ return x.first <= y.first; };
//
//    // OPEN List
//    std::set<std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
//    active_vertices.insert({0, source});
//    while (not active_vertices.empty())
//    {
//        Key where = active_vertices.begin()->second;
//        if (where == target)  // target found
//        {
//            auto p = orderPath(previous, source, target);
//            p = decimate_path(p);  // reduce size of path to half
//            return p;
//        }
//        active_vertices.erase(active_vertices.begin());
//        for (auto ed : neighboors_8(where))
//        {
//            //qInfo() << __FUNCTION__ << min_distance[ed.second.id] << ">" << min_distance[fmap.at(where).id] << "+" << ed.second.cost;
//            if (min_distance[ed.second.id] > min_distance[fmap.at(where).id] + ed.second.cost)
//            {
//                active_vertices.erase({min_distance[ed.second.id], ed.first});
//                min_distance[ed.second.id] = min_distance[fmap.at(where).id] + ed.second.cost;
//                previous[ed.second.id] = std::make_pair(fmap.at(where).id, where);
//                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
//                //active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
//            }
//        }
//    }
//    qInfo() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") to (" <<  target_.x() << "," << target_.y() << ") not  found. Returning empty path";
    return std::list<QPointF>();
};
std::vector<Eigen::Vector2f> Local_Grid::compute_path(const QPointF &source_, const QPointF &target_)
{
//    auto lpath = computePath(source_, target_);
//    std::vector<Eigen::Vector2f> path(lpath.size());
//    for(auto &&[i, p] : lpath | iter::enumerate)
//        path[i] = Eigen::Vector2f(p.x(), p.y());
//    return  path;
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors(const Local_Grid::Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs,
                                                                              bool all)
{
    std::vector<std::pair<Key, T>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz]: iter::zip(xincs, zincs))
    {
        //Key lk{k.ang + itx, k.rad + itz};
        Key key = pointToKey(k.ang + itx, k.rad + itz);
        auto &&[success, p] = getCell(key);
        if (not success) continue;

        if (all)
            neigh.emplace_back(std::make_pair(key, p));
        else // only free cells
            if (p.free)
                neigh.emplace_back(std::make_pair(key, p));
    }
    return neigh;
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors_8(const Local_Grid::Key &k, bool all)
{
    const int &I = angle_dim.step;
    const int &J = radius_dim.step;
    static const std::vector<int> ang_incs = {I, I, I, 0, -I, -I, -I, 0};
    static const std::vector<int> rad_incs = {J, 0, -J, -J, -J, 0, J, J};
    return this->neighboors(k, ang_incs, rad_incs, all);
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors_16(const Local_Grid::Key &k, bool all)
{
    const int &I = angle_dim.step;
    const int &J = radius_dim.step;
    static const std::vector<int> ang_incs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
    static const std::vector<int> rad_incs = {2*J, 2*J, 2*J,  J,   0 , -J , -2*J, -2*J,-2*J,-2*J,-2*J, -J, 0,J, 2*J, 2*J};
    return this->neighboors(k, ang_incs, rad_incs, all);
}
/**
 @brief Recovers the optimal path from the list of previous nodes
*/
std::list<QPointF> Local_Grid::orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
{
    std::list<QPointF> res;
//    Key k = target;
//    std::uint32_t u = fmap.at(k).id;
//    while (previous[u].first != (std::uint32_t)-1)
//    {
//        res.push_front(QPointF(k.x, k.z));
//        u = previous[u].first;
//        k = previous[u].second;
//    }
//    //qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point
    return res;
};
std::list<QPointF> Local_Grid::decimate_path(const std::list<QPointF> &path)
{
    std::list<QPointF> res;
    for(auto &&p : iter::chunked(path,2))
        res.push_back(p[0]);
    return res;
}
inline double Local_Grid::heuristicL2(const Key &a, const Key &b) const
{
    //return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
    return 0.0;
}

/////////////////////////////// COSTS /////////////////////////////////////////////////////////
void Local_Grid::update_costs(bool wide)
{
    for(auto &&[k,v] : iter::filter([](auto v){ return std::get<1>(v).cost > 1;}, fmap))
    {
        v.tile->setFreeColor();
        v.cost = 1.f;
    }

    //update grid values
    if(wide)
    {
        for (auto &&[k, v]: iter::filterfalse([](auto v) { return std::get<1>(v).free; }, fmap))
        {
            v.cost = 100;
            v.tile->setOccupiedColor(0);
            for (auto neighs = neighboors_16(k); auto &&[kk, vv]: neighs)
            {
                fmap.at(kk).cost = 100;
                fmap.at(kk).tile->setOccupiedColor(0);
            }
        }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 100; }, fmap))
            for (auto neighs = neighboors_8(k); auto &&[kk, vv]: neighs)
            {
                if (vv.cost < 100)
                {
                    fmap.at(kk).cost = 50;
                    fmap.at(kk).tile->setOccupiedColor(1);
                }
            }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 50; }, fmap))
            for (auto neighs = neighboors_8(k); auto &&[kk, vv]: neighs)
            {
                if (vv.cost < 50)
                {
                    fmap.at(kk).cost = 25;
                    fmap.at(kk).tile->setOccupiedColor(2);
                }
            }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 25; }, fmap))
            for (auto neighs = neighboors_8(k); auto &[kk, vv]: neighs)
            {
                if (vv.cost < 25)
                {
                    fmap.at(kk).cost = 15;
                    fmap.at(kk).tile->setOccupiedColor(3);
                }
            }
    }
    else  // not wide
    {
        for (auto &&[k, v]: iter::filterfalse([](auto v) { return std::get<1>(v).free; }, fmap))
        {
            v.cost = 100;
            v.tile->setOccupiedColor(0);
            fmap.at(k).cost = 100;
            fmap.at(k).tile->setOccupiedColor(0);
        }
    }
}

/////////////////////////////// UPDATE /////////////////////////////////////////////////////////
void Local_Grid::update_map_from_polar_data( const std::vector<Eigen::Vector2f> &points, float max_laser_range)
{
    Eigen::Vector2f paux;
    //qInfo() << __FUNCTION__ << points.size() << "points";
    for(const auto &point : points) // point.x() = angle; point.y() = radius
    {
        const float &ang = point.x();
        const float &dist = point.y();
        int num_steps = ceil(dist/(radius_dim.step));
        paux = {ang, 0.f};
        for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
        {
            paux.y() = dist*step;
            add_miss(paux);
        }
        if(dist < max_laser_range)
            add_hit(point);

        if((paux-point).norm() < radius_dim.step)  // in case last miss overlaps tip
            add_hit(point);
    }
    //update_costs(true);
}
void Local_Grid::update_map_from_3D_points(  std::shared_ptr<std::vector<std::tuple<float, float, float>>> points)
{

}

/////////////////////////////// AUX /////////////////////////////////////////////////////////
bool Local_Grid::is_path_blocked(const std::vector<Eigen::Vector2f> &path) // grid coordinates
{
    for(const auto &p: path)
        if(is_occupied(p) or get_cost(p)>=50)
           return true;
    return false;
}

void Local_Grid::clear()
{
    for (const auto &[key, value]: fmap)
        scene->removeItem(value.tile);
    fmap.clear();
}

////////////////////////////// NEIGHS /////////////////////////////////////////////////////////
std::optional<QPointF> Local_Grid::closestMatching_spiralMove(const QPointF &p, std::function<bool(std::pair<Local_Grid::Key, Local_Grid::T>)> pred)
{
    size_t moveUnit = TILE_SIZE;
    int vi = moveUnit, vj = 0, tamSegmento = 1, i = p.x(), j = p.y(), recorrido = 0;

    QPointF retPoint;
    while(true)
    {
        i += vi; j += vj; ++recorrido;
        retPoint.setX(i); retPoint.setY(j);
        Key key = pointToKey(retPoint);
        const auto &[success, v] = getCell(key);
        if(success and pred(std::make_pair(key, v)))
            return std::optional<QPointF>(retPoint);
        if (recorrido == tamSegmento) {
            recorrido = 0;
            int aux = vi; vi = -vj; vj = aux;
            if (vj == 0)
                ++tamSegmento;
        }
    }
}
std::optional<QPointF> Local_Grid::closest_obstacle(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return not cell.second.free; });
}
std::optional<QPointF> Local_Grid::closest_free(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return cell.second.free; });
}
std::optional<QPointF> Local_Grid::closest_free_4x4(const QPointF &p)
{
//    return this->closestMatching_spiralMove(p, [this, p](const auto &cell){
//        if (not cell.second.free)
//            return false;
//        Key key = pointToKey(QPointF(cell.first.x, cell.first.z));
//        std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> L1 = neighboors_16(key, false);
//        return (L1.size() == 16);
//    });
}
std::tuple<bool, QVector2D> Local_Grid::vectorToClosestObstacle(QPointF center)
{
//    QTime reloj = QTime::currentTime();
//    qDebug()<<" reloj "<< reloj.restart();
//    qDebug()<< "Computing neighboors of " << center;
//    auto k = pointToKey(center.x(), center.y());
//    QVector2D closestVector;
//    bool obstacleFound = false;
//
//    auto neigh = neighboors_8(k, true);
//    float dist = std::numeric_limits<float>::max();
//    for (auto n : neigh)
//    {
//        if (n.second.free == false)
//        {
//            QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x,n.first.z)) ;
//            if (vec.length() < dist)
//            {
//                dist = vec.length();
//                closestVector = vec;
//            }
//            qDebug() << __FUNCTION__ << "Obstacle found";
//            obstacleFound = true;
//        }
//    }
//
//    if (!obstacleFound)
//    {
//        auto DistNeigh = neighboors_16(k, true);
//        for (auto n : DistNeigh)
//        {
//            if (n.second.free == false)
//            {
//                QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x, n.first.z)) ;
//                if (vec.length() < dist)
//                {
//                    dist = vec.length();
//                    closestVector = vec;
//                }
//                obstacleFound = true;
//            }
//        }
//    }
//    return std::make_tuple(obstacleFound,closestVector);
}

auto operator<<(std::ostream &os, const Local_Grid::Key &k) -> decltype(k.save(os), os)
{
    k.save(os);
    return os;
};
auto operator>>(std::istream &is, Local_Grid::Key &k) -> decltype(k.read(is), is)
{
    k.read(is);
    return is;
};
auto operator<<(std::ostream &os, const Local_Grid::T &t) -> decltype(t.save(os), os)
{
    t.save(os);
    return os;
};
auto operator>>(std::istream &is, Local_Grid::T &t) -> decltype(t.read(is), is)
{
    t.read(is);
    return is;
};