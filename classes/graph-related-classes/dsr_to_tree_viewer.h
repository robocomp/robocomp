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

#ifndef DSR_TO_TREE_VIEWER_H
#define DSR_TO_TREE_VIEWER_H

#include <chrono>
#include <QWidget>
#include <QTreeWidget>
#include <QMouseEvent>
#include "CRDT.h"

class GraphNode;
class GraphEdge;

namespace DSR
{
    class DSRtoTreeViewer : public QTreeWidget
    {
        Q_OBJECT
        public:
            DSRtoTreeViewer(std::shared_ptr<CRDT::CRDTGraph> G_, QTreeWidget *parent=0);
            std::shared_ptr<CRDT::CRDTGraph> getGraph()  			  	{return G;};
		     
        public slots:   // From G
            void add_or_assign_node_SLOT(const std::int32_t id, const std::string &type);
            void add_or_assign_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string& type);
			void del_edge_SLOT(const std::int32_t from, const std::int32_t to,  const std::string &edge_tag);
			void del_node_SLOT(int id);
            
        protected:  
            virtual void wheelEvent(QWheelEvent* event);
            virtual void resizeEvent(QResizeEvent* event);
        private:
            std::shared_ptr<CRDT::CRDTGraph> G;
            void createGraph();
    };
};
#endif

