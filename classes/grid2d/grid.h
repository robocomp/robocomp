/*
 * Copyright 2018 <copyright holder> <email>
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
 * limitations under the License.
 */

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <fstream>
#include <cppitertools/zip.hpp>
#include <cppitertools/range.hpp>
#include <limits>
#include <QGraphicsScene>
#include <chrono>

class Grid
{
    using Myclock = std::chrono::system_clock;
    using Msec = std::chrono::duration<double, std::milli>;
    using Seconds = std::chrono::seconds;

    public:
        using Dimensions = QRectF;
        float TILE_SIZE;
        struct Key
        {
            long int x;
            long int z;
            public:
                Key() : x(0), z(0){};
                Key(long int &&x, long int &&z) : x(std::move(x)), z(std::move(z))
                    {  };
                Key(long int &x, long int &z) : x(x), z(z){  };
                Key(const long int &x, const long int &z) : x(x), z(z){ };
                Key(const QPointF &p)
                {
                    x = p.x();
                    z = p.y();
                };
                QPointF toQPointF() const { return QPointF(x,z);};
                bool operator==(const Key &other) const
                {
                    return x == other.x && z == other.z;
                };
                void save(std::ostream &os) const   { os << x << " " << z << " "; }; //method to save the keys
                void read(std::istream &is)         { is >> x >> z; };					   //method to read the keys
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
            bool free;
            bool visited;
            float cost;
            std::string node_name;
            // method to save the value
            void save(std::ostream &os) const {	os << free << " " << visited << " " << node_name; };
            void read(std::istream &is) {	is >> free >> visited >> node_name;};
        };
        using FMap = std::unordered_map<Key, T, KeyHasher>;
        Dimensions dim;

        void initialize( bool read_from_file = true,
                         const std::string &file_name = std::string(),
                         std::uint16_t num_threads = 10);
        std::tuple<bool, T&> getCell(long int x, long int z);
        std::tuple<bool, T&> getCell(const Key &k);
        T at(const Key &k) const                            { return fmap.at(k);};
        T &at(const Key &k)                                 { return fmap.at(k);};
        typename FMap::iterator begin()                     { return fmap.begin(); };
        typename FMap::iterator end()                       { return fmap.end(); };
        typename FMap::const_iterator begin() const         { return fmap.begin(); };
        typename FMap::const_iterator end() const           { return fmap.begin(); };
        size_t size() const                                 { return fmap.size(); };
        template <typename Q>
        void insert(const Key &key, const Q &value);
        void clear();
        void saveToFile(const std::string &fich);
        void readFromFile(const std::string &fich);
        std::string saveToString() const;
        void readFromString(const std::string &cadena);
        std::list<QPointF> computePath(const QPointF &source_, const QPointF &target_);
        Key pointToGrid(long int x, long int z) const;
        Key pointToGrid(const QPointF &p) const;
        void setFree(const Key &k);
        bool isFree(const Key &k) ;
        bool cellNearToOccupiedCellByObject(const Key &k, const std::string &target_name);
        void setOccupied(const Key &k);
        void setCost(const Key &k,float cost);
        void markAreaInGridAs(const QPolygonF &poly, bool free);   // if true area becomes free
        void modifyCostInGrid(const QPolygonF &poly, float cost);
        std::optional<QPointF> closest_obstacle(const QPointF &p);
        std::optional<QPointF> closest_free(const QPointF &p);
        std::optional<QPointF> closest_free_4x4(const QPointF &p);
        std::tuple<bool, QVector2D> vectorToClosestObstacle(QPointF center);
        std::vector<std::pair<Key, T>> neighboors(const Key &k, const std::vector<int> &xincs,const std::vector<int> &zincs, bool all = false);
        std::vector<std::pair<Key, T>> neighboors_8(const Key &k,  bool all = false);
        std::vector<std::pair<Key, T>> neighboors_16(const Key &k,  bool all = false);
        void draw(QGraphicsScene* scene);

    private:
        FMap fmap;
        std::vector<QGraphicsRectItem *> scene_grid_points;
        std::list<QPointF> orderPath(const std::vector<std::pair<std::uint32_t, Key>> &previous, const Key &source, const Key &target);
        inline double heuristicL2(const Key &a, const Key &b) const;
        std::optional<QPointF> closestMatching_spiralMove(const QPointF &p, std::function<bool(std::pair<Grid::Key, Grid::T>)> pred);
};

#endif // GRID_H
