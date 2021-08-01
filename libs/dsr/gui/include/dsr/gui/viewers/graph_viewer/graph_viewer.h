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

#include <dsr/api/dsr_api.h>
#include <dsr/gui/viewers/_abstract_graphic_view.h>

#include <chrono>
#include <QWidget>

#include <QMouseEvent>
#include <QGLWidget>
#include <QResizeEvent>
#include <QMenu>


class GraphNode;
class GraphEdge;

namespace DSR
{
    class GraphViewer : public AbstractGraphicViewer
    {
        Q_OBJECT
        public:
            GraphViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent=0);
			~GraphViewer();
            std::shared_ptr<DSR::DSRGraph> getGraph()  			  	{return G;};
			std::map<std::uint64_t , GraphNode*> getGMap() const 			{return gmap;};
            QGraphicsEllipseItem* getCentralPoint() const 				{return central_point;};


        public slots:
    		// From G
            void add_or_assign_node_SLOT(std::uint64_t id, const std::string &type);
            void add_or_assign_edge_SLOT(std::uint64_t from, std::uint64_t to, const std::string& type);
			void del_edge_SLOT(std::uint64_t from, std::uint64_t to,  const std::string &edge_tag);
			void del_node_SLOT(uint64_t id);
			void hide_show_node_SLOT(uint64_t id, bool visible);
			// Others
			void toggle_animation(bool state);
			void reload(QWidget * widget);


        protected:
            std::shared_ptr<DSR::DSRGraph> G;
            GraphNode* new_visual_node(uint64_t id, const std::string &type, const std::string &name, bool debug = false);
            GraphEdge* new_visual_edge(GraphNode *sourceNode, GraphNode *destNode, const QString &edge_name);
            GraphEdge* new_visual_edge(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
        private:
			std::shared_ptr<GraphViewer> own;

            std::map<std::uint64_t, GraphNode*> gmap;
			std::map<std::tuple<std::uint64_t, std::uint64_t, std::string>, GraphEdge*> gmap_edges;
			QGraphicsEllipseItem *central_point;
			QMenu *contextMenu, *showMenu;
			std::map<std::string,std::set<std::uint64_t>> type_id_map;
			int timerId = 0;
            void showContextMenu(QMouseEvent *event);


    	protected:
            void createGraph();
			virtual void timerEvent(QTimerEvent *event);
			virtual void mousePressEvent(QMouseEvent *event);


    };
};
#endif

