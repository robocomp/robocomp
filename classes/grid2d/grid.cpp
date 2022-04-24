#include "grid.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/slice.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <cppitertools/filterfalse.hpp>

auto operator<<(std::ostream &os, const Grid::Key &k) -> decltype(k.save(os), os)
{
    k.save(os);
    return os;
};
auto operator>>(std::istream &is, Grid::Key &k) -> decltype(k.read(is), is)
{
    k.read(is);
    return is;
};
auto operator<<(std::ostream &os, const Grid::T &t) -> decltype(t.save(os), os)
{
    t.save(os);
    return os;
};
auto operator>>(std::istream &is, Grid::T &t) -> decltype(t.read(is), is)
{
    t.read(is);
    return is;
};


void Grid::initialize(  QRectF dim_,
                        int tile_size,
                        QGraphicsScene *scene_,
                        bool read_from_file,
                        const std::string &file_name,
                        QPointF grid_center,
                        float grid_angle)
{
    static QGraphicsRectItem *bounding_box = nullptr;
    dim = dim_;
    TILE_SIZE = tile_size;
    scene = scene_;
    qInfo() << __FUNCTION__ <<  "World dimension: " << dim << TILE_SIZE << "I assume that Y+ axis goes upwards";
    qInfo() << __FUNCTION__ <<  "World dimension: ";
    qInfo() << "    " << "left:" << dim.left() << "right:" << dim.right() << "bottom:" << dim.bottom() << "top:" << dim.top() << "tile:" << TILE_SIZE;
    /// CHECK DIMENSIONS BEFORE PROCEED
    qInfo() << __FUNCTION__ << "Grid coord" << grid_center << grid_angle;
    for (const auto &[key, value]: fmap)
        scene->removeItem(value.tile);
    if(bounding_box != nullptr) scene->removeItem(bounding_box);
    fmap.clear();

//    if(read_from_file and not file_name.empty())
//        readFromFile(file_name);
    QColor my_color = QColor("White");
    //my_color.setAlpha(40);
    std::uint32_t id=0;
    Eigen::Matrix2f matrix;
    matrix << cos(grid_angle) , -sin(grid_angle) , sin(grid_angle) , cos(grid_angle);
    std::cout << matrix << std::endl;
    for (float i = dim.left(); i < dim.right(); i += TILE_SIZE)
        for (float j = dim.top(); j < dim.bottom(); j += TILE_SIZE)
        {
            T aux;
            aux.id = id++;
            aux.free = true;
            aux.visited = false;
            aux.cost = 1.0;
            QGraphicsRectItem* tile = scene->addRect(-TILE_SIZE/2, -TILE_SIZE/2, TILE_SIZE, TILE_SIZE, QPen(my_color), QBrush(my_color));
            tile->setZValue(10);
            auto res = matrix * Eigen::Vector2f(i, j) + Eigen::Vector2f(grid_center.x(), grid_center.y());
            tile->setPos(res.x(), res.y());
            tile->setRotation(qRadiansToDegrees(grid_angle));
            aux.tile = tile;
            insert(Key(i, j), aux);
            //qInfo() << __FUNCTION__ << i << j << aux.id << aux.free << aux.tile->pos();
        }

    // create cost matrix
    costs = cv::Mat::ones(dim.height(), dim.width(), CV_8UC1);   // rows, cols

    // draw bounding box
    bounding_box = scene->addRect(dim, QPen(QColor("Grey"), 40));
    bounding_box->setPos(grid_center);
    bounding_box->setZValue(12);
    bounding_box->setRotation(qRadiansToDegrees(grid_angle));
}
void Grid::insert(const Key &key, const T &value)
{
    fmap.insert(std::make_pair(key, value));
}
// FIX THIS
std::tuple<bool, Grid::T&> Grid::getCell(long int x, long int z)
{
    return getCell(pointToGrid(x,z));
}
std::tuple<bool, Grid::T&> Grid::getCell(const Key &k)  //overladed version
{
    if (not dim.contains(QPointF(k.x, k.z)))
        return std::forward_as_tuple(false, T());
    else
      try
      {
          return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x, k.z)));
      }
      catch(const std::exception &e)
      {
          //qWarning() << __FUNCTION__ << " No key found in grid: (" << k.x << k.z << ")";
          return std::forward_as_tuple(false, T());
      }
}
Grid::Key Grid::pointToGrid(long int x, long int z) const
{
    // bottom is top since Y axis is inverted
    int kx = rint((x - dim.left()) / TILE_SIZE);
    int kz = rint((z - dim.top()) / TILE_SIZE);
    return Key(dim.left() + kx * TILE_SIZE, dim.top() + kz * TILE_SIZE);
};
Grid::Key Grid::pointToGrid(const QPointF &p) const
{
    int kx = rint((p.x() - dim.left()) / TILE_SIZE);
    int kz = rint((p.y() - dim.top()) / TILE_SIZE);
    return Key(dim.left() + kx * TILE_SIZE, dim.top() + kz * TILE_SIZE);
};
////////////////////////////////////////////////////////////////////////////////
void Grid::saveToFile(const std::string &fich)
{
    std::ofstream myfile;
    myfile.open(fich);
    for (const auto &[k, v] : fmap)
        myfile << k << v << std::endl;

    myfile.close();
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements written to " << fich << std::endl;
}
std::string Grid::saveToString() const
{
    std::ostringstream stream;
    for (const auto &[k, v] : fmap)
        stream << k << v << v.cost << std::endl;

    std::cout << "Grid::" << __FUNCTION__ << " " << fmap.size() << " elements written to osdtringstream";
    return stream.str();
}
void Grid::readFromString(const std::string &cadena)
{
    fmap.clear();

    std::istringstream stream(cadena);
    std::string line;
    std::uint32_t count = 0;
    while ( std::getline (stream, line) )
    {
        //std::cout << line << std::endl;
        std::stringstream ss(line);
        int x, z;
        bool free, visited;
        float cost;
        std::string node_name;
        ss >> x >> z >> free >> visited >> cost>> node_name;
        fmap.emplace(pointToGrid(x, z), T{count++, free, false, cost});
    }
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements read from "  << std::endl;
}
void Grid::readFromFile(const std::string &fich)
{
    std::ifstream myfile(fich);
    std::string line;
    std::uint32_t count = 0;
    if (!myfile)
    {
        std::cout << fich << " No file found" << std::endl;
        std::terminate();
    }
    while ( std::getline (myfile, line) )
    {
        //std::cout << line << std::endl;
        std::stringstream ss(line);
        int x, z;
        bool free, visited;
        std::string node_name;
        ss >> x >> z >> free >> visited >> node_name;
        fmap.emplace(pointToGrid(x, z), T{count++, free, false, 1.f});
    }
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements read from " << fich << std::endl;
}
///////////////////////////////////////////////////////////////////////////////
//deprecated
bool Grid::isFree(const Key &k)
{
    const auto &[success, v] = getCell(k);
    if(success)
        return v.free;
    else
        return false;
}
//deprecated
void Grid::setFree(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.free = true;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor("white")));
    }
}
void Grid::set_free(int cx, int cy)
{
    setFree(pointToGrid(cx, cy));
}
void Grid::set_free(const QPointF &p)
{
    auto x = static_cast<long int>(p.x());
    auto y = static_cast<long int>(p.y());
    set_free(x, y);
}
void Grid::set_free(float xf, float yf)
{
    auto x = static_cast<long int>(xf);
    auto y = static_cast<long int>(yf);
    set_free(x, y);
}
void Grid::set_free(long int x, long int y)
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
void Grid::setOccupied(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.free = false;
//        if(v.tile != nullptr)
//            v.tile->setBrush(QBrush(QColor(params.occupied_color)));
    }
}
void Grid::setOccupied(long int x, long int y)
{
    auto &&[success, v] = getCell(x,y);
    if(success)
    {
        v.free = false;
//        if(v.tile != nullptr)
//            v.tile->setBrush(QBrush(QColor("red")));
    }
}
void Grid::setOccupied(const QPointF &p)
{
    setOccupied((long int)p.x(), (long int)p.y());
}
void Grid::add_miss(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.misses++;
        if((float)v.hits/(v.hits+v.misses) < params.occupancy_threshold)
        {
            if(not v.free)
                this->flipped++;
            v.free = true;
            //v.tile->setBrush(QBrush(QColor(params.free_color)));
        }
        v.misses = std::clamp(v.misses, 0.f, 10.f);
        this->updated++;
    }
//    else
//        qWarning() << __FUNCTION__ << "Cell not found" << "[" << p.x() << p.y() << "]";
}
void Grid::add_hit(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.hits++;
        //qInfo() << __FUNCTION__ << v.hits << (float)v.hits/(v.hits+v.misses);
        if((float)v.hits/(v.hits+v.misses) >= params.occupancy_threshold)
        {
            if(v.free)
                this->flipped++;
            v.free = false;
            //v.tile->setBrush(QBrush(QColor(params.occupied_color)));
        }
        v.hits = std::clamp(v.hits, 0.f, 10.f);
        this->updated++;
    }
}
float Grid::percentage_changed()
{
    return (flipped / updated);
}
void Grid::setVisited(const Key &k, bool visited)
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
bool Grid::is_visited(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
        return v.visited;
    else
        return false;
}
void Grid::setCost(const Key &k,float cost)
{
    auto &&[success, v] = getCell(k);
    if(success)
        v.cost = cost;
}
void Grid::set_all_costs(float value)
{
    for(auto &[key, cell] : fmap)
        cell.cost = value;
}
int Grid::count_total() const
{
    return fmap.size();
}
int Grid::count_total_visited() const
{
    int total = 0;
    for(const auto &[k, v] : fmap)
        if(v.visited)
            total ++;
    return total;
}
void Grid::set_all_to_not_visited()
{
    for(auto &[k,v] : fmap)
       setVisited(k, false);
}
void Grid::set_all_to_free()
{
    for(auto &[k,v] : fmap)
        setFree(k);
}
// if true area becomes free
void Grid::markAreaInGridAs(const QPolygonF &poly, bool free)
{
    const qreal step = TILE_SIZE / 4;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
        {
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
            {
                if (free)
                    setFree(pointToGrid(x, y));
                else
                    setOccupied(pointToGrid(x, y));
            }
        }
}
void Grid::modifyCostInGrid(const QPolygonF &poly, float cost)
{
    const qreal step = TILE_SIZE / 4.f;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                setCost(pointToGrid(x, y),cost);
}
std::tuple<bool, QVector2D> Grid::vectorToClosestObstacle(QPointF center)
{
    QTime reloj = QTime::currentTime();
    qDebug()<<" reloj "<< reloj.restart();
    qDebug()<< "Computing neighboors of " << center;
    auto k = pointToGrid(center.x(),center.y());
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
///////////////////////////////////////////////////////////////////////////////////////////////////////
std::list<QPointF> Grid::computePath(const QPointF &source_, const QPointF &target_)
{
    //qInfo() << __FUNCTION__  << " from nose pos: " << source_ << " to " << target_ ;
    Key source = pointToGrid(source_.x(), source_.y());
    Key target = pointToGrid(target_.x(), target_.y());

    // Admission rules
    if (not dim.contains(target_))
    {
        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim << " Returning empty path";
        return std::list<QPointF>();
    }
    if (not dim.contains(source_))
    {
        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
        return std::list<QPointF>();
    }
    if (source == target)
    {
        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
        return std::list<QPointF>();
    }
    //source in a non-free cell (red cell)
    if(neighboors_8(source_).empty())
    {
        qInfo() << __FUNCTION__ << "Source on an occupied cell: " << source.x << ", " << source.z << "Returning empty path";
        std::optional<QPointF> new_source = closest_free(source_);
        source = pointToGrid(new_source->x(), new_source->y());
    }
    const auto &[success, val] = getCell(source);
    if(not success)
    {
//        qWarning() << "Could not find source position in Grid. Returning empty path";
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
std::vector<Eigen::Vector2f> Grid::compute_path(const QPointF &source_, const QPointF &target_)
{
    auto lpath = computePath(source_, target_);
    std::vector<Eigen::Vector2f> path(lpath.size());
    for(auto &&[i, p] : lpath | iter::enumerate)
        path[i] = Eigen::Vector2f(p.x(), p.y());
    return  path;
}

std::vector<std::pair<Grid::Key, Grid::T>> Grid::neighboors(const Grid::Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs, bool all)
{
    std::vector<std::pair<Key, T>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz]: iter::zip(xincs, zincs))
    {
        Key lk{k.x + itx, k.z + itz};
        const auto &[success, p] = getCell(lk);
        if (not success) continue;

        // check that incs are not both zero but have the same abs value, i.e. a diagonal
//        if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost == 1)
//            p.cost = 1.43;                                // if neighboor in diagonal, cost is sqrt(2)

        if (all)
            neigh.emplace_back(std::make_pair(lk, p));
        else // if all cells covered by the robot are free
        {
            bool all_free = true;
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
std::vector<std::pair<Grid::Key, Grid::T>> Grid::neighboors_8(const Grid::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
    static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};
    return this->neighboors(k, xincs, zincs, all);
}
std::vector<std::pair<Grid::Key, Grid::T>> Grid::neighboors_16(const Grid::Key &k, bool all)
{
    const int &I = TILE_SIZE;
    static const std::vector<int> xincs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
    static const std::vector<int> zincs = {2*I, 2*I, 2*I,  I,   0 , -I , -2*I, -2*I,-2*I,-2*I,-2*I, -I, 0,I, 2*I, 2*I};
    return this->neighboors(k, xincs, zincs, all);
}
/**
 @brief Recovers the optimal path from the list of previous nodes
*/
std::list<QPointF> Grid::orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
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
std::list<QPointF> Grid::decimate_path(const std::list<QPointF> &path)
{
    std::list<QPointF> res;
    for(auto &&p : iter::chunked(path,2))
        res.push_back(p[0]);
    return res;
}
inline double Grid::heuristicL2(const Key &a, const Key &b) const
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
}
void Grid::update_costs()
{
    set_all_costs(1);
    //update grid values
    for(auto &&[k,v] : iter::filterfalse([](auto v){ return std::get<1>(v).free;}, fmap))
    {
        v.cost = 100;
        v.tile->setBrush(QColor("Red"));
        // fill 16 adjacent cells
        for(auto neighs = neighboors_16(k); auto &[kk, vv] : neighs)
        {
            fmap.at(kk).cost = 100;
            vv.tile->setBrush(QColor("Red"));
        }
    }
    for(auto &&[k,v] : iter::filter([](auto v){ return std::get<1>(v).cost==100;}, fmap))
        for(auto neighs = neighboors_8(k); auto &[kk, vv] : neighs)
        {
            if(vv.cost < 100)
            {
                fmap.at(kk).cost = 50;
                vv.tile->setBrush(QColor("Orange"));
            }
        }
    for(auto &&[k,v] : iter::filter([](auto v){ return std::get<1>(v).cost==50;}, fmap))
        for(auto neighs = neighboors_8(k); auto &[kk, vv] : neighs)
        {
            if(vv.cost < 50)
            {
                fmap.at(kk).cost = 25;
                vv.tile->setBrush(QColor("Yellow"));
            }
        }
}
///////////////////////////////////////////////////////////////////////////////////////
void Grid::draw()
{
    //clear previous points
    for (QGraphicsRectItem* item : scene_grid_points)
        scene->removeItem((QGraphicsItem*)item);

    scene_grid_points.clear();
    //create new representation
    std::string color;
    for( const auto &[key,value] : fmap)
    {
        if(value.free)
        {
            if (value.cost == 2.0) //affordance spaces
                color = "#FFFF00";
            else if (value.cost == 3.0) //lowvisited spaces
                color = "#FFBF00";
            else if (value.cost == 4.0) //mediumvisited spaces
                color = "#FF8000";
            else if (value.cost == 5.0) //highVisited spaces
                color = "#FF4000";
            else if (value.cost == 8.0) //zona social
                color = "#BF00FF";
            else if (value.cost == 10.0) //zona personal
                color = "#00BFFF";
            else if (value.cost == 50.0) //Affordance maximum
                color = "#FF0000";
            else
                color = "White";
        }
        else // occupied
            color = "Red";

        QColor my_color = QColor(QString::fromStdString(color));
        my_color.setAlpha(40);
        QGraphicsRectItem* aux = scene->addRect(-TILE_SIZE/2, -TILE_SIZE/2, TILE_SIZE, TILE_SIZE, QPen(my_color), QBrush(my_color));
        aux->setZValue(1);
        aux->setPos(key.x, key.z);
        scene_grid_points.push_back(aux);
    }
}
void Grid::clear()
{
    for (const auto &[key, value]: fmap)
        scene->removeItem(value.tile);
    fmap.clear();
}

std::optional<QPointF> Grid::closestMatching_spiralMove(const QPointF &p, std::function<bool(std::pair<Grid::Key, Grid::T>)> pred)
{
    size_t moveUnit = TILE_SIZE;
    int vi = moveUnit, vj = 0, tamSegmento = 1, i = p.x(), j = p.y(), recorrido = 0;

    QPointF retPoint;
    while(true)
    {
        i += vi; j += vj; ++recorrido;
        retPoint.setX(i); retPoint.setY(j);
        Key key = pointToGrid(retPoint);
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

std::optional<QPointF> Grid::closest_obstacle(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return not cell.second.free; });
}

std::optional<QPointF> Grid::closest_free(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [](auto cell){ return cell.second.free; });
}

std::optional<QPointF> Grid::closest_free_4x4(const QPointF &p)
{
    return this->closestMatching_spiralMove(p, [this, p](const auto &cell){
        if (not cell.second.free)
            return false;
        Key key = pointToGrid(QPointF(cell.first.x, cell.first.z));
        std::vector<std::pair<Grid::Key, Grid::T>> L1 = neighboors_16(key, false);
        return (L1.size() == 16);
    });
}
