#include "grid2d.h"
#include <QVector2D>
#include <QGraphicsRectItem>
#include <cppitertools/enumerate.hpp>

template <typename T>
void Grid<T>::initialize(QGraphicsScene* scene,
                         Dimensions dim_,
                         bool read_from_file,
                         const std::string &file_name)
{
    qDebug() << __FUNCTION__ << "FileName:" << QString::fromStdString(file_name);
    uint count = 0;
    dim = dim_;
    qInfo() << __FUNCTION__ << dim.HMIN << dim.WIDTH << dim.VMIN << dim.HEIGHT;
    fmap.clear();
    if(read_from_file and not file_name.empty())
        readFromFile(file_name);
    else
    {
        QColor free_color_t(free_color); free_color_t.setAlpha(40);
        for (int i = dim.HMIN; i < dim.HMIN + dim.WIDTH; i += dim.TILE_SIZE)
            for (int j = dim.VMIN; j < dim.VMIN + dim.HEIGHT; j += dim.TILE_SIZE)
            {
                T t;
                Key k(i, j);
                t.id = count++; t.free = true; t.visited = false; t.x = i; t.y = j;
                t.g_item = scene->addRect(i, j, dim.TILE_SIZE, dim.TILE_SIZE, QPen(free_color_t), QBrush(QColor(free_color_t)));
                fmap.emplace(k, t);
            }
        if(not file_name.empty())
            saveToFile(file_name);
    }
}

template <typename T>
std::tuple<bool, T &> Grid<T>::getCell(long int x, long int z)
{
    if (!(x >= dim.HMIN and x < dim.HMIN + dim.WIDTH and z >= dim.VMIN and z < dim.VMIN + dim.HEIGHT))
        return std::forward_as_tuple(false, T());
    else
        return std::forward_as_tuple(true, fmap.at(pointToGrid(x, z)));
}

template <typename T>
std::tuple<bool, T &> Grid<T>::getCell(const Key &k) //overladed version
{
    if (!(k.x >= dim.HMIN and k.x < dim.HMIN + dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
        return std::forward_as_tuple(false, T());
    else
        return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x, k.z)));
}

template <typename T>
typename Grid<T>::Key Grid<T>::pointToGrid(long int x, long int z) const
{
    int kx = (x - dim.HMIN) / dim.TILE_SIZE;
    int kz = (z - dim.VMIN) / dim.TILE_SIZE;
    return Key(dim.HMIN + kx * dim.TILE_SIZE, dim.VMIN + kz * dim.TILE_SIZE);
};

template <typename T>
void Grid<T>::fill_with_obstacles(std::vector<QPolygonF> world_obstacles)
{
    size_t cont = 0;
    qInfo() << world_obstacles.size() << fmap.size();
    for( auto &[key, cell] : fmap)
    {
        for (const auto &obs : world_obstacles)
        {
            if(obs.containsPoint(QPointF(cell.x, cell.y), Qt::OddEvenFill))
            {
                cell.free = false;
                cell.g_item->setBrush(QColor(occupied_color));
                continue;
            }
        }
        // qInfo() << __FUNCTION__ << "Progress: " << cont++ * 100 / fmap.size() << "%";
    }
    for( auto &[key, cell] : fmap)
    {
        if(not cell.free)
            cont++;
    }
    qInfo() << cont;
}

////////////////////////////////////////////////////////////////////////////////

template <typename T>
void Grid<T>::saveToFile(const std::string &fich)
{
    std::ofstream myfile;
    myfile.open(fich);
    for (auto &[k, v] : fmap)
    {
        myfile << k << v << std::endl;
    }
    myfile.close();
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements written to " << fich << std::endl;
}

template <typename T>
void Grid<T>::readFromFile(const std::string &fich)
{
    std::ifstream myfile(fich);
    std::string line;
    std::uint32_t count = 0;
    while ( std::getline (myfile, line) )
    {
        //std::cout << line << std::endl;
        std::stringstream ss(line);
        int x, z;
        bool free, visited;
        std::string node_name;
        ss >> x >> z >> free >> visited >> node_name;
        fmap.emplace(pointToGrid(x, z), T{count++, free, false, 1.f, node_name});
    }
    std::cout << __FUNCTION__ << " " << fmap.size() << " elements read from " << fich << std::endl;
}

template <typename T>
std::list<QPointF> Grid<T>::computePath(const QPointF &source_, const QPointF &target_)
{
    qInfo() << __FUNCTION__;
    Key source = pointToGrid(source_.x(), source_.y());
    Key target = pointToGrid(target_.x(), target_.y());

    // Admission rules
    if (!(target.x >= dim.HMIN and target.x < dim.HMIN + dim.WIDTH and target.z >= dim.VMIN and target.z < dim.VMIN + dim.HEIGHT))
    {
        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim.HMIN << dim.VMIN << dim.HMIN+dim.WIDTH << dim.VMIN+dim.HEIGHT
        << "Returning empty path";
        return std::list<QPointF>();
    }
    if (!(source.x >= dim.HMIN and source.x < dim.HMIN + dim.WIDTH and source.z >= dim.VMIN and source.z < dim.VMIN + dim.HEIGHT))
    {
        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
        return std::list<QPointF>();
    }
    if (source == target)
    {
        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
        return std::list<QPointF>();
    }
    // vector de distancias inicializado a DBL_MAX
    std::vector<double> min_distance(fmap.size(),std::numeric_limits<double>::max());
    // std::uint32_t id with source value
    const auto &[success, val] = getCell(source);
    if(not success)
    {
        qWarning() << "Could not find source position in Grid";
        return std::list<QPointF>();
    }
    auto id = val.id;
    // initialize source position to 0
    min_distance[id] = 0;
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

    while (!active_vertices.empty())
    {
        Key where = active_vertices.begin()->second;
        if (where == target)
        {
//				qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point
            auto p = orderPath(previous, source, target);
            if (p.size() > 1)
                return p;
            else
                return std::list<QPointF>();
        }
        active_vertices.erase(active_vertices.begin());
        for (auto ed : neighboors_8(where))
        {
//				qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
            if (min_distance[ed.second.id] > min_distance[fmap[where].id] + ed.second.cost)
            {
                active_vertices.erase({min_distance[ed.second.id], ed.first});
                min_distance[ed.second.id] = min_distance[fmap[where].id] + ed.second.cost;
                previous[ed.second.id] = std::make_pair(fmap[where].id, where);
                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
                // active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
            }
        }
    }
    qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
    return std::list<QPointF>();
};

template <typename T>
bool Grid<T>::isFree(const Key &k)
{
   const auto &[success, v] = getCell(k);
   if(success)
       return v.free;
   else
        return false;
}

template <typename T>
void Grid<T>::setFree(const Key &k)
{
    auto [success, v] = getCell(k);
    if(success)
        v.free = true;
}

template <typename T>
bool Grid<T>::cellNearToOccupiedCellByObject(const Key &k, const std::string &target_name)
{
    auto neigh = this->neighboors_8(k, true);
    for(const auto &[key, val] : neigh)
        if(val.free==false and val.node_name==target_name)
            return true;
    return false;
}

template <typename T>
void Grid<T>::setOccupied(const Key &k)
{
    auto [success, v] = getCell(k);
    if(success)
        v.free = false;
}

template <typename T>
void Grid<T>::setCost(const Key &k,float cost)
{
    auto &[success, v] = getCell(k);
    if(success)
        v.cost = cost;
}

// if true area becomes free
template <typename T>
void Grid<T>::markAreaInGridAs(const QPolygonF &poly, bool free)
{
    const qreal step = dim.TILE_SIZE / 4;
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

template <typename T>
void Grid<T>::modifyCostInGrid(const QPolygonF &poly, float cost)
{
    const qreal step = dim.TILE_SIZE / 4;
    QRectF box = poly.boundingRect();
    for (auto &&x : iter::range(box.x() - step / 2, box.x() + box.width() + step / 2, step))
        for (auto &&y : iter::range(box.y() - step / 2, box.y() + box.height() + step / 2, step))
            if (poly.containsPoint(QPointF(x, y), Qt::OddEvenFill))
                setCost(pointToGrid(x, y),cost);
}

template <typename T>
std::tuple<bool, QVector2D> Grid<T>::vectorToClosestObstacle(QPointF center)
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

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors(const Grid<T>::Key &k, const std::vector<int> xincs,const std::vector<int> zincs, bool all)
{
    std::vector<std::pair<Key, T>> neigh;
    // list of increments to access the neighboors of a given position
    for (auto &&[itx, itz] : iter::zip(xincs, zincs))
    {
        Key lk{k.x + itx, k.z + itz};
        const auto &[success, p] = getCell(lk);
        if(not success) continue;

        // check that incs are not both zero but have the same abs value, i.e. a diagonal
        if (itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)) and p.cost==1)
            p.cost = 1.41; 								// if neighboor in diagonal, cost is sqrt(2)

        if(all)
            neigh.emplace_back(std::make_pair(lk, p));
        else
            if (p.free)
                neigh.emplace_back(std::make_pair(lk, p));
    }
    return neigh;
}

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors_8(const Grid<T>::Key &k, bool all)
{
    const int &I = dim.TILE_SIZE;
    static const std::vector<int> xincs = {I, I, I, 0, -I, -I, -I, 0};
    static const std::vector<int> zincs = {I, 0, -I, -I, -I, 0, I, I};
    return this->neighboors(k, xincs, zincs, all);
}

template <typename T>
std::vector<std::pair<typename Grid<T>::Key, T>> Grid<T>::neighboors_16(const Grid<T>::Key &k, bool all)
{
    const int &I = dim.TILE_SIZE;
    static const std::vector<int> xincs = {0,   I,   2*I,  2*I, 2*I, 2*I, 2*I, I, 0, -I, -2*I, -2*I,-2*I,-2*I,-2*I, -I};
    static const std::vector<int> zincs = {2*I, 2*I, 2*I,  I,   0 , -I , -2*I, -2*I,-2*I,-2*I,-2*I, -I, 0,I, 2*I, 2*I};
    return this->neighboors(k, xincs, zincs, all);
}

/**
 @brief Recovers the optimal path from the list of previous nodes
*/
template <typename T>
std::list<QPointF> Grid<T>::orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target)
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

template <typename T>
inline double Grid<T>::heuristicL2(const Key &a, const Key &b) const
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.z - b.z) * (a.z - b.z));
}

template <typename T>
void Grid<T>::draw(QGraphicsScene* scene)
{
    QColor f_color(free_color); f_color.setAlpha(50);
    QColor o_color(occupied_color); o_color.setAlpha(10);
    for (const auto &[key, value] : fmap)
    {
        if (value.free)
            value.g_item->setBrush(f_color);
        else
            value.g_item->setBrush(o_color);
    }

//        //my_color.setAlpha(60);
//        QGraphicsRectItem* aux = scene->addRect(key.x, key.z, 50, 50, QPen(my_color), QBrush(my_color));
//        aux->setZValue(1);
//        scene_grid_points.push_back(aux);
//    }
}

template <typename T>
void Grid<T>::draw_path(QGraphicsScene *scene, const std::list<QPointF> &path, uint size)
{
    static std::vector<QGraphicsEllipseItem *> path_paint;
    static QString path_color = "DarkBlue";;
    for(auto p : path_paint)
        scene->removeItem(p);
    path_paint.clear();
    for(auto &&[e, p] : iter::enumerate(path))
        path_paint.push_back(scene->addEllipse(p.x()-size/2, p.y()-size/2, size , size, QPen(path_color), QBrush(QColor(path_color))));
}

template <typename T>
void Grid<T>::clear()
{
    fmap.clear();
}

template <class T>


/////////////////////////////////////////////////////////////////////7777
auto operator<<(std::ostream &os, const T &t) -> decltype(t.save(os), os)
{
    t.save(os);
    return os;
};
template <class T>
auto operator>>(std::istream &is, T &t) -> decltype(t.read(is), is)
{
    t.read(is);
    return is;
};
