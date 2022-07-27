#include "local_grid.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/slice.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <cppitertools/filterfalse.hpp>

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

    // clean existing map
    for (const auto &[key, value]: fmap)
        scene->removeItem(value.tile);
    fmap.clear();

    QColor my_color = QColor("White");
    int id = 0;
    for (float ang = angle_dim.init; ang < angle_dim.end; ang += angle_dim.step)
        for (float rad = radius_dim.init; rad < radius_dim.end; rad += radius_dim.step)
        {
            T aux;
            aux.id = id++;
            aux.free = true;
            aux.visited = false;
            aux.cost = 1.0;
            //QGraphicsRectItem* tile = scene->addRect(-TILE_SIZE/2, -TILE_SIZE/2, TILE_SIZE, TILE_SIZE, QPen(my_color), QBrush(my_color));
            // polar to cart
            float i = rad*sin(ang);
            float j = rad*cos(ang);
            //tile->setPos(i, j);
            //aux.tile = tile;
            insert(Key(i, j), aux);
        }

    // draw outlines
   for(auto &e : lines)
       scene->removeItem(e);
   for(auto &c : circles)
       scene->removeItem(c);
   QColor outline_color("blue");
   for (float ang = angle_dim.init; ang < angle_dim.end; ang += angle_dim.step*5)
        lines.push_back(scene->addLine(0, 0, radius_dim.end*sin(ang), radius_dim.end*cos(ang), QPen(outline_color, 30)));

   for (float rad = radius_dim.init; rad < radius_dim.end; rad += radius_dim.step*5)
   {
       float i = rad*sin(-3.f*M_PI/2.f);
       float j = rad*cos(-3.f*M_PI/2.f);
       //circles.push_back(scene->addEllipse(i, j, rad*2, rad*2, QPen(outline_color)));
   }

   // create cost matrix
   costs = cv::Mat::ones(angle_dim.size(), radius_dim.size(), CV_8UC1);   // rows, cols

}
void Local_Grid::insert(const Key &key, const T &value)
{
    fmap.insert(std::make_pair(key, value));
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(long int x, long int z)
{
    if (not dim.contains(QPointF(x, z)))
        return std::forward_as_tuple(false, T());
    else
        try
        {
            return std::forward_as_tuple(true, fmap.at(pointToKey(x, z)));
        }
        catch(const std::exception &e)
        {
            //qWarning() << __FUNCTION__ << " No key found in grid: (" << k.x << k.z << ")";
            return std::forward_as_tuple(false, T());
        }
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(const Key &k)
{
    if (not dim.contains(k.toQPointF()))
        return std::forward_as_tuple(false, T());
    else
      try
      {
          return std::forward_as_tuple(true, fmap.at(pointToKey(k.x, k.z)));
      }
      catch(const std::exception &e)
      {
          //qWarning() << __FUNCTION__ << " No key found in grid: (" << k.x << k.z << ")";
          return std::forward_as_tuple(false, T());
      }
}
inline std::tuple<bool, Local_Grid::T&> Local_Grid::getCell(const Eigen::Vector2f &p)
{
    if (not dim.contains(QPointF(p.x(), p.y())))
        return std::forward_as_tuple(false, T());
    else
        try
        {
            return std::forward_as_tuple(true, fmap.at(pointToKey(p.x(), p.y())));
        }
        catch(const std::exception &e)
        {
            //qWarning() << __FUNCTION__ << " No key found in grid: (" << k.x << k.z << ")";
            return std::forward_as_tuple(false, T());
        }
}
Local_Grid::Key Local_Grid::pointToKey(long int x, long int z) const
{
    // bottom is top since Y axis is inverted
    int kx = rint((x - dim.left()) / TILE_SIZE);
    int kz = rint((z - dim.top()) / TILE_SIZE);
    return Key(dim.left() + kx * TILE_SIZE, dim.top() + kz * TILE_SIZE);
};
Local_Grid::Key Local_Grid::pointToKey(const QPointF &p) const
{
    int kx = rint((p.x() - dim.left()) / TILE_SIZE);
    int kz = rint((p.y() - dim.top()) / TILE_SIZE);
    return Key(dim.left() + kx * TILE_SIZE, dim.top() + kz * TILE_SIZE);
};
Local_Grid::Key Local_Grid::pointToKey(const Eigen::Vector2f &p) const
{
    int kx = rint((p.x() - dim.left()) / TILE_SIZE);
    int kz = rint((p.y() - dim.top()) / TILE_SIZE);
    return Key(dim.left() + kx * TILE_SIZE, dim.top() + kz * TILE_SIZE);
};

//////////////////////////////// STATUS //////////////////////////////////////////
//deprecated
bool Local_Grid::isFree(const Key &k)
{
    const auto &[success, v] = getCell(k);
    if(success)
        return v.free;
    else
        return false;
}
bool Local_Grid::is_occupied(const Eigen::Vector2f &p)
{
    const auto &[success, v] = getCell(p.x(),p.y());
    if(success)
        return not v.free;
    else
        return true;  // non existing cells are returned as occupied
}
//deprecated
void Local_Grid::setFree(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.free = true;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor("white")));
    }
}
void Local_Grid::set_free(int cx, int cy)
{
    setFree(pointToKey(cx, cy));
}
void Local_Grid::set_free(const QPointF &p)
{
    auto x = static_cast<long int>(p.x());
    auto y = static_cast<long int>(p.y());
    set_free(x, y);
}
void Local_Grid::set_free(float xf, float yf)
{
    auto x = static_cast<long int>(xf);
    auto y = static_cast<long int>(yf);
    set_free(x, y);
}
void Local_Grid::set_free(long int x, long int y)
{
    auto &&[success, v] = getCell(x, y);
    if(success)
    {
        v.free = true;
        if (v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor(params.free_color)));
    }
}
//deprecated
void Local_Grid::setOccupied(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.free = false;
//        if(v.tile != nullptr)
//            v.tile->setBrush(QBrush(QColor(params.occupied_color)));
    }
}
void Local_Grid::setOccupied(long int x, long int y)
{
    auto &&[success, v] = getCell(x,y);
    if(success)
    {
        v.free = false;
//        if(v.tile != nullptr)
//            v.tile->setBrush(QBrush(QColor("red")));
    }
}
void Local_Grid::setOccupied(const QPointF &p)
{
    setOccupied((long int)p.x(), (long int)p.y());
}
void Local_Grid::add_miss(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.misses++;
        if((float)v.hits/(v.hits+v.misses) < params.occupancy_threshold)
        //if((float)v.hits/(v.hits+v.misses) < params.prob_free)
        {
            if(not v.free)
                this->flipped++;
            v.free = true;
            //v.tile->setBrush(QBrush(QColor(params.free_color)));
        }
        v.misses = std::clamp(v.misses, 0.f, 20.f);
        this->updated++;
    }
//    else
//        qWarning() << __FUNCTION__ << "Cell not found" << "[" << p.x() << p.y() << "]";
}
void Local_Grid::add_hit(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.hits++;
        if((float)v.hits/(v.hits+v.misses) >= params.occupancy_threshold)
        //if((float)v.hits/(v.hits+v.misses) >= params.prob_occ)
            {
            if(v.free)
                this->flipped++;
            v.free = false;
            //v.tile->setBrush(QBrush(QColor(params.occupied_color)));
        }
        v.hits = std::clamp(v.hits, 0.f, 20.f);
        this->updated++;
    }
}
void Local_Grid::log_update(const Eigen::Vector2f &p, float prob)
{
    static double TRESHOLD_P_FREE = log_odds(0.3);
    static double TRESHOLD_P_OCC = log_odds(0.6);

    // update probability matrix using inverse sensor model
    auto &&[success, v] = getCell(p);
    if(success)
    {
        v.log_odds += log_odds(prob);
        qInfo() << __FUNCTION__ << v.log_odds;
        auto r = retrieve_p(v.log_odds);
        if (r < TRESHOLD_P_FREE)
        {
            v.free = true;
            v.tile->setBrush(QColor("White"));
        }
        else if (r > TRESHOLD_P_OCC)
        {
            v.free = false;
            v.tile->setBrush(QColor("Red"));
        }
    }
}
double Local_Grid::log_odds(double prob)
{
    // Log odds ratio of p(x):
    //              p(x)
    // l(x) = log ----------
    //              1 - p(x)
    return log(prob / (1 - prob));
}
double Local_Grid::retrieve_p(double l)
{
    // Retrieve p(x) from log odds ratio:
    //                   1
    // p(x) = 1 - ---------------
    //             1 + exp(l(x))

    return 1 - 1 / (1 + exp(l));
}
float Local_Grid::percentage_changed()
{
    return (flipped / updated);
}
void Local_Grid::setVisited(const Key &k, bool visited)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.visited = visited;
        if(visited)
           v.tile->setBrush(QColor("Orange"));
        else
            v.tile->setBrush(QColor("White"));
    }
}
bool Local_Grid::is_visited(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
        return v.visited;
    else
        return false;
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
int Local_Grid::count_total_visited() const
{
    int total = 0;
    for(const auto &[k, v] : fmap)
        if(v.visited)
            total ++;
    return total;
}
void Local_Grid::set_all_to_not_visited()
{
    for(auto &[k,v] : fmap)
       setVisited(k, false);
}
void Local_Grid::set_all_to_free()
{
    for(auto &[k,v] : fmap)
        setFree(k);
}
void Local_Grid::markAreaInGridAs(const QPolygonF &poly, bool free)
{
    const qreal step = TILE_SIZE / 4;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
        {
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
            {
                if (free)
                    setFree(pointToKey(x, y));
                else
                    setOccupied(pointToKey(x, y));
            }
        }
}
void Local_Grid::modifyCostInGrid(const QPolygonF &poly, float cost)
{
    const qreal step = TILE_SIZE / 4.f;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                setCost(pointToKey(x, y), cost);
}

////////////////////////////////////// PATH //////////////////////////////////////////////////////////////
std::list<QPointF> Local_Grid::computePath(const QPointF &source_, const QPointF &target_)
{
    //qInfo() << __FUNCTION__  << " from nose pos: " << source_ << " to " << target_ ;
    Key source = pointToKey(source_.x(), source_.y());
    Key target = pointToKey(target_.x(), target_.y());

    // Admission rules
    if (not dim.contains(target_))
    {
        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim << " Returning empty path";
        return {};
    }
    if (not dim.contains(source_))
    {
        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
        return {};
    }
    if (source == target)
    {
        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
        return {};
    }
    //source in a non-free cell (red cell)
    if(neighboors_8(source_).empty())
    {
        qInfo() << __FUNCTION__ << "Source on an occupied cell: " << source.x << ", " << source.z << "Returning empty path";
        std::optional<QPointF> new_source = closest_free(source_);
        source = pointToKey(new_source->x(), new_source->y());
    }
    const auto &[success, val] = getCell(source);
    if(not success)
    {
//        qWarning() << "Could not find source position in Local_Grid. Returning empty path";
        return std::list<QPointF>();
    }

    // vector de distancias inicializado a UINT_MAX
    std::vector<uint32_t> min_distance(fmap.size(), std::numeric_limits<uint32_t>::max());
    // initialize source position to 0
    min_distance[val.id] = 0;
    // vector de pares<std::uint32_t, Key> initialized to (-1, Key())
    std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
    // lambda to compare two vertices: a < b if a.id<b.id or
    auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y){ return x.first <= y.first; };

    // OPEN List
    std::set<std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
    active_vertices.insert({0, source});
    while (not active_vertices.empty())
    {
        Key where = active_vertices.begin()->second;
        if (where == target)  // target found
        {
            auto p = orderPath(previous, source, target);
            p = decimate_path(p);  // reduce size of path to half
            return p;
        }
        active_vertices.erase(active_vertices.begin());
        for (auto ed : neighboors_8(where))
        {
            //qInfo() << __FUNCTION__ << min_distance[ed.second.id] << ">" << min_distance[fmap.at(where).id] << "+" << ed.second.cost;
            if (min_distance[ed.second.id] > min_distance[fmap.at(where).id] + ed.second.cost)
            {
                active_vertices.erase({min_distance[ed.second.id], ed.first});
                min_distance[ed.second.id] = min_distance[fmap.at(where).id] + ed.second.cost;
                previous[ed.second.id] = std::make_pair(fmap.at(where).id, where);
                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
                //active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
            }
        }
    }
    qInfo() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") to (" <<  target_.x() << "," << target_.y() << ") not  found. Returning empty path";
    return std::list<QPointF>();
};
std::vector<Eigen::Vector2f> Local_Grid::compute_path(const QPointF &source_, const QPointF &target_)
{
    auto lpath = computePath(source_, target_);
    std::vector<Eigen::Vector2f> path(lpath.size());
    for(auto &&[i, p] : lpath | iter::enumerate)
        path[i] = Eigen::Vector2f(p.x(), p.y());
    return  path;
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors(const Local_Grid::Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs,
                                                            bool all)
{
    std::vector<std::pair<Key, T>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz]: iter::zip(xincs, zincs))
    {
        Key lk{k.x + itx, k.z + itz};
        auto &&[success, p] = getCell(lk);
        if (not success) continue;

        // check that incs are not both zero but have the same abs value, i.e. a diagonal
//        if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost == 1)
//            p.cost = 1.43;                                // if neighboor in diagonal, cost is sqrt(2)

        if (all)
            neigh.emplace_back(std::make_pair(lk, p));
        else // if all cells covered by the robot are free
        {
            //bool all_free = true;
            if (p.free)
                neigh.emplace_back(std::make_pair(lk, p));
//            {
//                if(ceil(400.0/TILE_SIZE)<= 3) // robot occupies three cells, Check 8-neigh
//                {
//                    auto neigh = neighboors_8(lk, true);
//                    if( auto res = std::ranges::find_if_not(neigh, [](auto a){ return a.second.free;}); res != neigh.end())
//                        all_free = false;
////                    for (auto &&[fitx, fitz]: iter::zip(xincs, zincs))
////                    {
////                        Key flk{lk.x + fitx, lk.z + fitz};
////                        const auto &[fsuccess, fp] = getCell(flk);
////                        if (not fsuccess or not fp.free)
////                        {
////                            all_free = false;
////                            break;
////                        }
////                    }
//                }
//                else
//                {
//                    auto neigh = neighboors_16(lk, true);
//                    if( auto res = std::ranges::find_if_not(neigh, [](auto a){ return a.second.free;}); res != neigh.end())
//                        all_free = false;
//                }
//                if (all_free)
//                    neigh.emplace_back(std::make_pair(lk, p));
//            }
        }
    }
    return neigh;
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors_8(const Local_Grid::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
    static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};
    return this->neighboors(k, xincs, zincs, all);
}
std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> Local_Grid::neighboors_16(const Local_Grid::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
    static const std::vector<int> zincs = {2*I, 2*I, 2*I,  I,   0 , -I , -2*I, -2*I,-2*I,-2*I,-2*I, -I, 0,I, 2*I, 2*I};
    return this->neighboors(k, xincs, zincs, all);
}
/**
 @brief Recovers the optimal path from the list of previous nodes
*/
std::list<QPointF> Local_Grid::orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
{
    std::list<QPointF> res;
    Key k = target;
    std::uint32_t u = fmap.at(k).id;
    while (previous[u].first != (std::uint32_t)-1)
    {
        res.push_front(QPointF(k.x, k.z));
        u = previous[u].first;
        k = previous[u].second;
    }
    //qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point
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
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
}

/////////////////////////////// COSTS /////////////////////////////////////////////////////////
void Local_Grid::update_costs(bool wide)
{
    static QBrush free_brush(QColor(params.free_color));
    static QBrush occ_brush(QColor(params.occupied_color));
    static QBrush orange_brush(QColor("Orange"));
    static QBrush yellow_brush(QColor("Yellow"));
    static QBrush gray_brush(QColor("LightGray"));

    for(auto &&[k,v] : iter::filter([](auto v){ return std::get<1>(v).cost > 1;}, fmap))
    {
        v.tile->setBrush(free_brush);
        v.cost = 1.f;
    }

    //update grid values
    if(wide)
    {
        for (auto &&[k, v]: iter::filterfalse([](auto v) { return std::get<1>(v).free; }, fmap))
        {
            v.cost = 100;
            v.tile->setBrush(occ_brush);
            for (auto neighs = neighboors_16(k); auto &&[kk, vv]: neighs)
            {
                fmap.at(kk).cost = 100;
                fmap.at(kk).tile->setBrush(occ_brush);
            }
        }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 100; }, fmap))
            for (auto neighs = neighboors_8(k); auto &&[kk, vv]: neighs)
            {
                if (vv.cost < 100)
                {
                    fmap.at(kk).cost = 50;
                    fmap.at(kk).tile->setBrush(orange_brush);
                }
            }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 50; }, fmap))
            for (auto neighs = neighboors_8(k); auto &&[kk, vv]: neighs)
            {
                if (vv.cost < 50)
                {
                    fmap.at(kk).cost = 25;
                    fmap.at(kk).tile->setBrush(yellow_brush);
                }
            }
        for (auto &&[k, v]: iter::filter([](auto v) { return std::get<1>(v).cost == 25; }, fmap))
            for (auto neighs = neighboors_8(k); auto &[kk, vv]: neighs)
            {
                if (vv.cost < 25)
                {
                    fmap.at(kk).cost = 15;
                    fmap.at(kk).tile->setBrush(gray_brush);
                }
            }
    }
    else
    {
        for (auto &&[k, v]: iter::filterfalse([](auto v) { return std::get<1>(v).free; }, fmap))
        {
            v.cost = 100;
            v.tile->setBrush(occ_brush);
            fmap.at(k).cost = 100;
            fmap.at(k).tile->setBrush(occ_brush);
        }
    }
}
void Local_Grid::update_map_from_polar_data( const std::vector<Eigen::Vector2f> &points, float max_laser_range)
{
//    for(const auto &point : points) // point.x() = radius; point.y() = angle
//    {
//        int num_steps = ceil(point.x()/(TILE_SIZE));
//        Eigen::Vector2f p;
//        for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
//        {
//            p = point*step;
//            add_miss(p);
//        }
//        if(length <= max_laser_range)
//            add_hit(point);
//
//        if((p-point).norm() < TILE_SIZE)  // in case last miss overlaps tip
//            add_hit(point);
//    }
}
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
    return this->closestMatching_spiralMove(p, [this, p](const auto &cell){
        if (not cell.second.free)
            return false;
        Key key = pointToKey(QPointF(cell.first.x, cell.first.z));
        std::vector<std::pair<Local_Grid::Key, Local_Grid::T>> L1 = neighboors_16(key, false);
        return (L1.size() == 16);
    });
}
std::tuple<bool, QVector2D> Local_Grid::vectorToClosestObstacle(QPointF center)
{
    QTime reloj = QTime::currentTime();
    qDebug()<<" reloj "<< reloj.restart();
    qDebug()<< "Computing neighboors of " << center;
    auto k = pointToKey(center.x(), center.y());
    QVector2D closestVector;
    bool obstacleFound = false;

    auto neigh = neighboors_8(k, true);
    float dist = std::numeric_limits<float>::max();
    for (auto n : neigh)
    {
        if (n.second.free == false)
        {
            QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x,n.first.z)) ;
            if (vec.length() < dist)
            {
                dist = vec.length();
                closestVector = vec;
            }
            qDebug() << __FUNCTION__ << "Obstacle found";
            obstacleFound = true;
        }
    }

    if (!obstacleFound)
    {
        auto DistNeigh = neighboors_16(k, true);
        for (auto n : DistNeigh)
        {
            if (n.second.free == false)
            {
                QVector2D vec = QVector2D(QPointF(k.x, k.z)) - QVector2D(QPointF(n.first.x, n.first.z)) ;
                if (vec.length() < dist)
                {
                    dist = vec.length();
                    closestVector = vec;
                }
                obstacleFound = true;
            }
        }
    }
    return std::make_tuple(obstacleFound,closestVector);
}

