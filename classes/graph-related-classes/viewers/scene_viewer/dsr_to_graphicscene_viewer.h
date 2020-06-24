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

#ifndef DSR_TO_GRAPHCISCENE_VIEWER_H
#define DSR_TO_GRAPHCISCENE_VIEWER_H

#include "../../CRDT.h"
#include "../_abstract_graphic_view.h"

#include <math.h>
#include <filesystem>
		
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsItem>
#include <QMouseEvent>
#include <QGLWidget>
#include <QScrollBar>



namespace DSR
{

    class DSRtoGraphicsceneViewer : public AbstractGraphicViewer
    {
        Q_OBJECT

        private:
            const float ROBOT_LENGTH = 400;
            std::vector<std::vector<float>> cube_positions = {{0.5,0.5,0.5}, {0.5, 0.5,-0.5}, {0.5, -0.5,0.5}, {0.5, -0.5, -0.5}, {-0.5, 0.5, 0.5}, {-0.5, 0.5, -0.5}, {-0.5, -0.5, 0.5}, {-0.5, -0.5, -0.5} };        
            std::list<std::string> no_drawable_childs = {"omniRobot", "differentialRobot", "person"};
            std::shared_ptr<CRDT::CRDTGraph> G;
            std::unique_ptr<CRDT::InnerAPI> innermodel;

            std::map<int, QGraphicsItem*> scene_map;
            std::map<std::string,std::vector<int>> edge_map;
            std::set<int> ignore_nodes;
        public:
            DSRtoGraphicsceneViewer(std::shared_ptr<CRDT::CRDTGraph> G_, QWidget *parent=0);


        public slots:   // From G
            void add_or_assign_node_slot(const std::int32_t id, const std::string &type);
            void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type);
            void del_node_slot(const std::int32_t id);
            void del_edge_slot(const std::int32_t from, const std::int32_t to, const std::string &edge_tag);

        private:
            void create_graph();
            std::list<int> get_parent_list(std::int32_t node_id);
            void update_edge_chain(std::list<int> parent_list);
            void get_2d_projection(std::string node_name, std::vector<int> size, QPolygon &polygon, int &zvalue);
            bool is_drawable(std::list<int> parent_list);
            bool check_RT_required_attributes(Node node);

            void add_or_assign_plane(Node &node);
            void add_or_assign_person(Node &node);
            void add_or_assign_mesh(Node &node);     
            void add_or_assign_robot(Node &node);

            
            void update_scene_object_pose(std::int32_t node_id);
    };
};
#endif

