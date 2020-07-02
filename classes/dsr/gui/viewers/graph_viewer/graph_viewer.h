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

#ifndef DSR_TO_GRAPH_VIEWER_H
#define DSR_TO_GRAPH_VIEWER_H

#include "../../../api/dsr_api.h"
#include "../_abstract_graphic_view.h"

#include <chrono>
#include <QWidget>

#include <QMouseEvent>
#include <QGLWidget>
#include <QResizeEvent>


class GraphNode;
class GraphEdge;

namespace DSR
{
    class DSRtoGraphViewer : public AbstractGraphicViewer
    {
        Q_OBJECT
        public:
            DSRtoGraphViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent=0);
			~DSRtoGraphViewer();
            std::shared_ptr<DSR::DSRGraph> getGraph()  			  	{return G;};
			std::map<std::int32_t, GraphNode*> getGMap() const 			{return gmap;};
            QGraphicsEllipseItem* getCentralPoint() const 				{return central_point;};
			void itemMoved();


        public slots:   // From G
            void add_or_assign_node_SLOT(const std::int32_t id, const std::string &type);
            void add_or_assign_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string& type);
			void del_edge_SLOT(const std::int32_t from, const std::int32_t to,  const std::string &edge_tag);
			void del_node_SLOT(int id);
			void hide_show_node_SLOT(int id, bool visible);
//            void save_graph_SLOT();		
//			void toggle_simulation_SLOT();


        private:
			std::shared_ptr<DSRtoGraphViewer> own;
            std::shared_ptr<DSR::DSRGraph> G;
            std::map<std::int32_t, GraphNode*> gmap;
			std::map<std::tuple<std::int32_t, std::int32_t, std::string>, GraphEdge*> gmap_edges;
			QGraphicsEllipseItem *central_point;
			int timerId = 0;
			bool do_simulate = false;
            void createGraph();


    	protected:
			virtual void timerEvent(QTimerEvent *event);
			virtual void mousePressEvent(QMouseEvent *event);


    };
};
#endif

