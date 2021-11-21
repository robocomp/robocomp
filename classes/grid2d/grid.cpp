#include "grid.h"
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/slice.hpp>
#include <execution>

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
                        std::uint16_t num_threads)
{
    dim = dim_;
    TILE_SIZE = tile_size;
    scene = scene_;
    qInfo() << __FUNCTION__ <<  "World dimension: " << dim << TILE_SIZE << "I assume that Y+ axis goes upwards";
    qInfo() << __FUNCTION__ <<  "World dimension: " << dim.left() << dim.right() << dim.bottom() << dim.top() << TILE_SIZE;
    /// CHECK DIMENSIONS BEFORE PROCEED
    fmap.clear();

//    if(read_from_file and not file_name.empty())
//        readFromFile(file_name);
    std::uint32_t id=0;
    for (float i = dim.left(); i < dim.right(); i += TILE_SIZE)
        for (float j = dim.top(); j < dim.bottom(); j += TILE_SIZE)
        {
            T aux;
            aux.id = id++;
            aux.free = true;
            aux.visited = true;
            aux.cost = 1.0;
            QColor my_color = QColor("LightGrey");
            my_color.setAlpha(40);
            QGraphicsRectItem* tile = scene->addRect(-TILE_SIZE/2, -TILE_SIZE/2, TILE_SIZE, TILE_SIZE, QPen(my_color), QBrush(my_color));
            tile->setZValue(1);
            tile->setPos(i, j);
            aux.tile = tile;
            insert(Key(i, j), aux);
            qInfo() << __FUNCTION__ << i << j << aux.id << aux.free << aux.tile->pos();
        }

//
//        auto duration = Myclock::now() - start_time;
//        std::cout << __FUNCTION__ << " " << count << " elements inserted.  It took " << std::chrono::duration_cast<std::chrono::seconds>(duration).count() << "secs" << std::endl;
//
//        if(not file_name.empty())
//            saveToFile(file_name);

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
typename Grid::Key Grid::pointToGrid(long int x, long int z) const
{
    int kx = (x - dim.left()) / TILE_SIZE;
    int kz = (z - dim.bottom()) / TILE_SIZE;
    return Key(dim.left() + kx * TILE_SIZE, dim.bottom() + kz * TILE_SIZE);
};
Grid::Key Grid::pointToGrid(const QPointF &p) const
{
    int kx = (p.x() - dim.left()) / TILE_SIZE;
    int kz = (p.y() - dim.bottom()) / TILE_SIZE;
    return Key(dim.left() + kx * TILE_SIZE, dim.bottom() + kz * TILE_SIZE);
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
bool Grid::isFree(const Key &k)
{
    const auto &[success, v] = getCell(k);
    if(success)
        return v.free;
    else
        return false;
}
void Grid::setFree(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
        v.free = true;
}
void Grid::setOccupied(const Key &k)
{
    auto &&[success, v] = getCell(k);
    if(success)
    {
        v.free = false;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor("red")));
    }
}
void Grid::setOccupied(long int x, long int y)
{
    auto &&[success, v] = getCell(x,y);
    if(success)
    {
        v.free = false;
        if(v.tile != nullptr)
            v.tile->setBrush(QBrush(QColor("red")));
    }
}
void Grid::setOccupied(const QPointF &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.free = false;
        v.tile->setBrush(QBrush(QColor("red")));
    }
}
void Grid::add_miss(const Eigen::Vector2f &p)
{
    auto &&[success, v] = getCell((long int)p.x(),(long int)p.y());
    if(success)
    {
        v.misses++;
        if((float)v.hits/(v.hits+v.misses) < params.occupancy_threshold)
        {
            if(not v.free) this->flipped++;
            v.free = true;
            v.tile->setBrush(QBrush(QColor(params.free_color)));
        }
        if (v.misses + v.hits == 20)
        {
            v.misses = 0;
            v.hits = 0;
        }
        this->updated++;
    }
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
            if(v.free) this->flipped++;
            v.free = false;
            v.tile->setBrush(QBrush(QColor(params.occupied_color)));
        }
        if (v.misses + v.hits == 20)
        {
            v.misses = 0;
            v.hits = 0;
        }
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
    qInfo() << __FUNCTION__  << " from nose pos: " << source_ << " to " << target_ ;
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
        qInfo() << __FUNCTION__ << "SE INICIA EL A* EN UN ROJO: " << source.x << ", " << source.z;
        std::optional<QPointF> new_source = closest_free(source_);
        source = pointToGrid(new_source->x(), new_source->y());
    }

    const auto &[success, val] = getCell(source);
    if(not success)
    {
        qWarning() << "Could not find source position in Grid";
        return std::list<QPointF>();
    }
    set_all_costs(1.0);

    // vector de distancias inicializado a UINT_MAX
    std::vector<uint32_t> min_distance(fmap.size(),std::numeric_limits<uint32_t>::max());
    // initialize source position to 0
    min_distance[val.id] = 0;
    // vector de pares<std::uint32_t,Key> initialized to (-1, Key())
    std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
    // lambda to compare two vertices: a < b if a.id<b.id or
    auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y) {
        if (x.first <= y.first)
            return true;
            //else if(x.first == y.first)
            //	return std::get<T&>(getCell(x.second)).id <= std::get<T&>(getCell(y.second)).id;
        else
            return false;
    };

    // OPEN List
    std::set<std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
    active_vertices.insert({0, source});
    while (not active_vertices.empty())
    {
        //qInfo() << __FILE__ << __LINE__ << "Entrando en while";
        Key where = active_vertices.begin()->second;
        if (where == target)
        {
            //qInfo() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point
            auto p = orderPath(previous, source, target);
            //qInfo() << "p.size() = " << p.size();
            //esto es solo pa cd encuentra un path eh xd
            if (p.size() > 1)
                return p;
            else
                return std::list<QPointF>();
        }
        //qInfo() << i++ << ": No where == target";
        active_vertices.erase(active_vertices.begin());
        for (auto ed : neighboors_8(where))
        {
            //qInfo() << __FUNCTION__ << min_distance[ed.second.id] << ">" << min_distance[fmap.at(where).id] << "+" << ed.second.cost;
            if (min_distance[ed.second.id] > min_distance[fmap.at(where).id] + ed.second.cost)
            {
                //qInfo() << "considerando este neighbor" << endl;
                active_vertices.erase({min_distance[ed.second.id], ed.first});
                min_distance[ed.second.id] = min_distance[fmap.at(where).id] + ed.second.cost;
                previous[ed.second.id] = std::make_pair(fmap.at(where).id, where);
                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
                // active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
            }
        }
    }
    qInfo() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") to (" <<  target_.x() << "," << target_.y() << ") not  found. Returning empty path";
    return std::list<QPointF>();
};
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
//            p.cost = 5;                                // if neighboor in diagonal, cost is sqrt(2)

        if (all)
            neigh.emplace_back(std::make_pair(lk, p));
        else // if all cells covered by the robot are free
        {
            bool all_free = true;
            if (p.free)
            {
                if(ceil(400.0/TILE_SIZE)<= 3) // robot occupies three cells, Check 8-neigh
                {
                    auto neigh = neighboors_8(lk, true);
                    if( auto res = std::ranges::find_if_not(neigh, [](auto a){ return a.second.free;}); res != neigh.end())
                        all_free = false;
//                    for (auto &&[fitx, fitz]: iter::zip(xincs, zincs))
//                    {
//                        Key flk{lk.x + fitx, lk.z + fitz};
//                        const auto &[fsuccess, fp] = getCell(flk);
//                        if (not fsuccess or not fp.free)
//                        {
//                            all_free = false;
//                            break;
//                        }
//                    }
                }
                else
                {
                    auto neigh = neighboors_16(lk, true);
                    if( auto res = std::ranges::find_if_not(neigh, [](auto a){ return a.second.free;}); res != neigh.end())
                        all_free = false;
                }
                if (all_free)
                    neigh.emplace_back(std::make_pair(lk, p));
            }
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
inline double Grid::heuristicL2(const Key &a, const Key &b) const
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
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
