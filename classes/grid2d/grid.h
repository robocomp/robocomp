/* Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <fstream>
#include <limits>
#include <tuple>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <chrono>
#include <unistd.h>
#include <QtCore>
#include <Eigen/Dense>
#include <QVector2D>
#include <QColor>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Grid
{
    using Myclock = std::chrono::system_clock;
    using Msec = std::chrono::duration<double, std::milli>;
    using Seconds = std::chrono::seconds;

public:
    using Dimensions = QRectF;
    int TILE_SIZE;

    struct Key
    {
        long int x;
        long int z;
    public:
        Key() : x(0), z(0)
        {};
        Key(long int &&x, long int &&z) : x(std::move(x)), z(std::move(z))
        {};
        Key(long int &x, long int &z) : x(x), z(z)
        {};
        Key(float &x, float &z) : x((long int) x), z((long int) z)
        {};
        Key(const long int &x, const long int &z) : x(x), z(z)
        {};
        Key(const QPointF &p)
        {
            x = p.x();
            z = p.y();
        };
        QPointF toQPointF() const
        { return QPointF(x, z); };

        bool operator==(const Key &other) const
        {
            return x == other.x && z == other.z;
        };

        void save(std::ostream &os) const
        { os << x << " " << z << " "; }; //method to save the keys
        void read(std::istream &is)
        { is >> x >> z; };                       //method to read the keys
    };

    struct KeyHasher
    {
        std::size_t operator()(const Key &k) const
        {
            using boost::hash_combine;
            using boost::hash_value;
            // Start with a hash value of 0    .
            std::size_t seed = 0;
            // Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
            hash_combine(seed, hash_value(k.x));
            hash_combine(seed, hash_value(k.z));
            return seed;
        };
    };

    struct T
    {
        std::uint32_t id;
        bool free = true;
        bool visited = false;
        float cost = 1;
        float hits = 0;
        float misses = 0;
        QGraphicsRectItem *tile;
        double log_odds = 0.0;  //log prior

        // method to save the value
        void save(std::ostream &os) const
        { os << free << " " << visited; };

        void read(std::istream &is)
        { is >> free >> visited; };
    };

    using FMap = std::unordered_map<Key, T, KeyHasher>;
    Dimensions dim = QRectF();

    void initialize(QRectF dim_,
                    int tile_size,
                    QGraphicsScene *scene,
                    bool read_from_file = true,
                    const std::string &file_name = std::string(),
                    QPointF grid_center = QPointF(0,0),
                    float grid_angle = 0.f);
    void clear();
    std::list<QPointF> computePath(const QPointF &source_, const QPointF &target_);
    std::vector<Eigen::Vector2f> compute_path(const QPointF &source_, const QPointF &target_);

    inline std::tuple<bool, T &> getCell(long int x, long int z);
    inline std::tuple<bool, T &> getCell(const Key &k);
    inline std::tuple<bool, T &> getCell(const Eigen::Vector2f &p);
//    T at(const Key &k) const
//    { return fmap.at(k); };
//    T &at(const Key &k)
//    { return fmap.at(k); };
    typename FMap::iterator begin()
    { return fmap.begin(); };
    typename FMap::iterator end()
    { return fmap.end(); };
    typename FMap::const_iterator begin() const
    { return fmap.begin(); };
    typename FMap::const_iterator end() const
    { return fmap.begin(); };
    size_t size() const
    { return fmap.size(); };
    void insert(const Key &key, const T &value);
    void saveToFile(const std::string &fich);
    void readFromFile(const std::string &fich);
    std::string saveToString() const;
    void readFromString(const std::string &cadena);
    Key pointToKey(long int x, long int z) const;
    Key pointToKey(const QPointF &p) const;
    Eigen::Vector2f pointToGrid(const Eigen::Vector2f &p) const;
    void setFree(const Key &k);
    void set_free(int cx, int cy);
    void set_free(const QPointF &p);
    void set_free(long int x, long int y);
    void set_free(float xf, float yf);
    bool isFree(const Key &k);
    void setVisited(const Key &k, bool visited);
    bool is_visited(const Key &k);
    void set_all_to_not_visited();
    void set_all_to_free();
    void setOccupied(const Key &k);
    void setOccupied(long int x, long int y);
    void setOccupied(const QPointF &p);
    void setCost(const Key &k, float cost);
    void add_miss(const Eigen::Vector2f &p);
    void add_hit(const Eigen::Vector2f &p);
    void log_update(const Eigen::Vector2f &p, float prob);
    double log_odds(double prob);
    double retrieve_p(double l);
    float percentage_changed();
    int count_total() const;
    int count_total_visited() const;
    void markAreaInGridAs(const QPolygonF &poly, bool free);   // if true area becomes free
    void modifyCostInGrid(const QPolygonF &poly, float cost);
    void update_costs(bool wide=true);
    std::optional<QPointF> closest_obstacle(const QPointF &p);
    std::optional<QPointF> closest_free(const QPointF &p);
    std::optional<QPointF> closest_free_4x4(const QPointF &p);
    std::tuple<bool, QVector2D> vectorToClosestObstacle(QPointF center);
    std::vector<std::pair<Key, T>> neighboors(const Key &k, const std::vector<int> &xincs, const std::vector<int> &zincs, bool all = false);
    std::vector<std::pair<Key, T>> neighboors_8(const Key &k, bool all = false);
    std::vector<std::pair<Key, T>> neighboors_16(const Key &k, bool all = false);
    void draw();

private:
    FMap fmap;
    QGraphicsScene *scene;
    std::vector<QGraphicsRectItem *> scene_grid_points;
    double updated=0.0, flipped=0.0;
    cv::Mat costs;

    std::list<QPointF> orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target);
    inline double heuristicL2(const Key &a, const Key &b) const;
    std::list<QPointF> decimate_path(const std::list<QPointF> &path);
    std::optional<QPointF> closestMatching_spiralMove(const QPointF &p, std::function<bool(std::pair<Grid::Key, Grid::T>)> pred);
    void set_all_costs(float value);

    struct Params
    {
        const QString free_color = "white";
        const QString occupied_color = "red";
        const float occupancy_threshold = 0.5;
    };
    Params params;
};
#endif // GRID_H
