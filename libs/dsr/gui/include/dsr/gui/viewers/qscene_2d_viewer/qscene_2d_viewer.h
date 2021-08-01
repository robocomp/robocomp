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

#include "dsr/api/dsr_api.h"
#include "dsr/gui/viewers/_abstract_graphic_view.h"
#include <math.h>
#include <filesystem>
		
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsItem>
#include <QMouseEvent>
#include <QGLWidget>
#include <QScrollBar>
#include <QGraphicsSceneHoverEvent>
#include <QInputDialog>

namespace DSR
{

    class QScene2dViewer : public AbstractGraphicViewer
    {
        Q_OBJECT
        private:
            QGraphicsItem *robot = nullptr;
            QGraphicsItem *laser_polygon = nullptr;
            bool delete_axis = false;
            QGraphicsRectItem *axis_center = nullptr, *axis_x = nullptr, *axis_y = nullptr;
            QMenu *contextMenu, *showMenu;
            bool drawlaser = false;
            bool drawpeople_space = false;
            bool drawaxis = false;
            const float ROBOT_LENGTH = 400;
            std::vector<std::vector<float>> cube_positions = {{0.5,0.5,0.5}, {0.5, 0.5,-0.5}, {0.5, -0.5,0.5}, {0.5, -0.5, -0.5}, {-0.5, 0.5, 0.5}, {-0.5, 0.5, -0.5}, {-0.5, -0.5, 0.5}, {-0.5, -0.5, -0.5} };        
            std::list<std::string> no_drawable_childs = {"omnirobot", "differentialrobot", "person"};
            std::shared_ptr<DSR::DSRGraph> G;
            std::unique_ptr<DSR::InnerEigenAPI> innermodel;

            std::map<uint64_t , QGraphicsItem*> scene_map;
            std::map<std::string,std::vector<uint64_t >> edge_map;
            std::set<uint64_t > ignore_nodes;
            std::map<uint64_t , std::string> orphand_nodes; //nodes without RT edge must be revisited
        public:
            QScene2dViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent=nullptr);
            ~QScene2dViewer() {
                qDebug() << "QScene2dViewer: " << G.use_count() ;

                if (delete_axis)
                {
                    delete axis_center;
                    delete axis_y;
                    delete axis_x;
                }
                G.reset();
            }
            void set_draw_laser(bool draw);
            void set_draw_people_spaces(bool draw);
            void set_draw_axis(bool draw);
            QGraphicsItem* get_robot_polygon() const { return robot;};
        public slots:   // From G
            void add_or_assign_node_slot(std::uint64_t  id, const std::string &type);
            void add_or_assign_edge_slot(std::uint64_t  from, std::uint64_t  to, const std::string& type);
            void del_node_slot(std::uint64_t  id);
            void del_edge_slot(std::uint64_t  from, std::uint64_t  to, const std::string &edge_tag);
            void reload(QWidget* widget);
        private:
            void create_graph();
            std::list<uint64_t > get_parent_list(std::uint64_t  node_id);
            void update_edge_chain(std::list<uint64_t > parent_list);
            void get_2d_projection(const std::string& node_name, std::vector<int> size, QPolygon &polygon, int &zvalue);
            bool is_drawable(std::list<uint64_t > parent_list);
            bool check_RT_required_attributes(Node node);

            void add_or_assign_plane(Node &node);
            void add_or_assign_person(Node &node);
            void add_or_assign_mesh(Node &node);     
            void add_or_assign_robot(Node &node);

            void add_or_assign_rect(Node &node, const std::string& color, const std::string& texture, int width, int height, int depth);
            
            void update_scene_object_pose(std::uint64_t  node_id);

            void draw_laser();
            void draw_axis();
            void draw_person_space(QGraphicsItem *sceneItem,Node &node);
            void draw_space(const std::string& name, const std::string& color_, int zvalue, Node &node, QGraphicsItem* parent);
            void showContextMenu(QMouseEvent *event);
        signals:
            void mouse_right_click(int pos_x, int pos_y, uint64_t  node_id);

        protected:
            virtual void mousePressEvent(QMouseEvent *event);
    };
};
#endif

