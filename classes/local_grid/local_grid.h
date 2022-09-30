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

#ifndef LOCAL_GRID_H
#define LOCAL_GRID_H

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
#include "qgraphicscellitem.h"
#include <ranges>
#include <timer/timer.h>


class Local_Grid
{
    using Myclock = std::chrono::system_clock;
    using Msec = std::chrono::duration<double, std::milli>;
    using Seconds = std::chrono::seconds;

public:
    using Dimensions = QRectF;
    struct Key
    {
        int ang;
        int rad;
        public:
            Key() : ang(0), rad(0) {};
            Key(int ang_, int rad_) : ang(ang_), rad(rad_) {};
            Key(const QPointF &p) : ang(p.x()), rad(p.y()) {};
            QPointF toQPointF() const { return QPointF(ang, rad); };

            bool operator==(const Key &other) const { return ang == other.ang && rad == other.rad; };
            void save(std::ostream &os) const
            { os << ang << " " << rad << " "; };         //method to save the keys
            void read(std::istream &is)
            { is >> ang >> rad; };                       //method to read the keys
            //float operator-(const Key &other) const { return (ang-other.ang)*(ang-other.ang)+(rad-other.rad)*(rad-other.rad);}
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
            hash_combine(seed, hash_value(k.ang));
            hash_combine(seed, hash_value(k.rad));
            return seed;
        };
    };

    // Cell data structure
    struct T
    {
        std::uint32_t id;
        bool free = true;
        bool visited = false;
        float cost = 1;
        float hits = 0;
        float misses = 0;
        QGraphicsCellItem *tile;
        double log_odds = 0.0;  //log prior

        // semantic elements
        int semantic_id;

        // method to save the value
        void save(std::ostream &os) const
        { os << free << " " << visited; };
        void read(std::istream &is)
        { is >> free >> visited; };
    };

    // Object data structure
    struct Object
    {
        int id;
        float ang;
        float dist;
        QGraphicsItem *bbox;
        std::int64_t timestamp;
    };
    std::map<int, Object> semantic_map;

    using FMap = std::unordered_map<Key, T, KeyHasher>;
    std::vector<Key> keys_vector;
    struct Ranges
    {
        float init = 0, end = 0, step = 0;
        std::size_t size() const { return init * end * step;}
    };
    int TILE_SIZE = 50;
    QRectF dim;
    Ranges angle_dim, radius_dim;

    void initialize( Ranges angle_dim_,
                     Ranges radius_dim_,
                     QGraphicsScene *scene_ = nullptr);
    void clear();
    std::list<QPointF> computePath(const QPointF &source_, const QPointF &target_);
    std::vector<Eigen::Vector2f> compute_path(const QPointF &source_, const QPointF &target_);
    void update_map_from_polar_data( const std::vector<Eigen::Vector2f> &points, float max_laser_range);
    void update_map_from_3D_points(const std::vector<std::tuple<float, float, float>> &points);
    void update_semantic_layer(float ang, float dist, int object, int type);
    bool is_path_blocked(const std::vector<Eigen::Vector2f> &path); // grid coordinates

    // Cell access
    inline std::tuple<bool, T &> getCell(int ang, int rad);  // deg, mm
    inline std::tuple<bool, T &> getCell(const Key &k);
    inline std::tuple<bool, T &> getCell(const Eigen::Vector2f &p);
    
    // Iterators
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

    // Access to content
    void insert(const Key &key, const T &value);
    Key pointToKey(int ang, int rad) const;
    Key pointToKey(const QPointF &p) const;
    Key pointToKey(const Eigen::Vector2f &p) const;
    Eigen::Vector2f pointToGrid(const Eigen::Vector2f &p) const;
    void set_free(const QPointF &p);
    void set_free(float ang, float yf);
    bool isFree(const Key &k);
    bool is_occupied(const Eigen::Vector2f &p);
    void setVisited(const Key &k, bool visited);
    bool is_visited(const Key &k);
    void set_all_to_not_visited();
    void set_all_to_free();
    void setOccupied(float ang, float rad);
    void setOccupied(const QPointF &p);
    void setCost(const Key &k, float cost);
    float get_cost(const Eigen::Vector2f &p);
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

    // Neighbours utilities
    std::optional<QPointF> closest_obstacle(const QPointF &p);
    std::optional<QPointF> closest_free(const QPointF &p);
    std::optional<QPointF> closest_free_4x4(const QPointF &p);
    std::tuple<bool, QVector2D> vectorToClosestObstacle(QPointF center);
    std::vector<std::pair<Key, T>> neighboors(const Key &k, const std::vector<int> &xincs, const std::vector<int> &zincs, bool all = false);
    std::vector<std::pair<Key, T>> neighboors_8(const Key &k, bool all = false);
    std::vector<std::pair<Key, T>> neighboors_16(const Key &k, bool all = false);


private:
    FMap fmap;
    QGraphicsScene *scene;
    std::vector<QGraphicsRectItem *> scene_grid_points;
    double updated=0.0, flipped=0.0;
    cv::Mat costs;

    std::list<QPointF> orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target);
    inline double heuristicL2(const Key &a, const Key &b) const;
    std::list<QPointF> decimate_path(const std::list<QPointF> &path);
    std::optional<QPointF> closestMatching_spiralMove(const QPointF &p, std::function<bool(std::pair<Local_Grid::Key, Local_Grid::T>)> pred);
    void set_all_costs(float value);
    inline float radians_to_degrees(float a) const { if(a>=0) return a*180.f/M_PI; else return a*180/M_PI+360;};

    struct Params
    {
        const QString free_color = "white";
        const QString occupied_color = "orange";
        const float occupancy_threshold = 0.5;
        const std::uint32_t max_object_unseen_timelife = 2000; //ms
    };
    Params params;

    QPixmap human_image, plant_image, chair_image;
};
#endif // LOCAL_GRID_H
