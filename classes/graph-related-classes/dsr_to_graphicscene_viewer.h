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

#include "CRDT.h"

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

    class DSRtoGraphicsceneViewer : public QGraphicsView
    {
        Q_OBJECT
        public:
            QGraphicsScene scene;
        private:
            std::shared_ptr<CRDT::CRDTGraph> G;
            std::unique_ptr<CRDT::InnerAPI> innermodel;      
            
            qreal m_scaleX, m_scaleY;
            bool _pan;
            int _panStartX, _panStartY;

            std::map<int, QGraphicsItem*> sceneMap;
            std::map<std::string,std::vector<int>> edgeMap;

        public:
            DSRtoGraphicsceneViewer(std::shared_ptr<CRDT::CRDTGraph> G_, float scaleX, float scaleY, QGraphicsView *parent=0);


        public slots:   // From G
            void add_or_assign_node_slot(const std::int32_t id, const std::string &type);
            void add_or_assign_edge_slot(const std::int32_t from, const std::int32_t to, const std::string& type);
            
        protected:  
            virtual void wheelEvent(QWheelEvent* event);
            virtual void mouseMoveEvent(QMouseEvent *event);
            virtual void mousePressEvent(QMouseEvent *event);
            virtual void mouseReleaseEvent(QMouseEvent *event);
            virtual void resizeEvent(QResizeEvent *event);

        private:
            void createGraph();
            void create_parent_list(std::int32_t node_id);
            void add_or_assign_box(Node &node);
            void add_or_assign_mesh(Node &node);     
            void add_or_assign_object(std::int32_t node_id, std::int32_t width, std::int32_t height, std::string node_name, std::string color, std::string filename);       
            void add_scene_rect(std::int32_t node_id, std::int32_t width, std::int32_t height, QVec pose, std::string color, std::string texture);
            void update_scene_rect_pose(std::int32_t node_id);
    };
};
#endif

