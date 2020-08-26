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
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QHBoxLayout>
#include "../../../api/dsr_api.h"
#include "../../../core/types/user_types.h"
class GraphNode;
class GraphEdge;

namespace DSR
{
    class TreeViewer : public QTreeWidget
    {
        Q_OBJECT
        public:
            TreeViewer(std::shared_ptr<DSR::DSRGraph> G_, QWidget *parent=0);
            std::shared_ptr<DSR::DSRGraph> getGraph()  			  	{return G;};
		     
        public slots:   // From G
            void add_or_assign_node_SLOT(const std::int32_t id, const std::string &type, const std::string &name = "");
			void add_or_assign_node_SLOT(const std::int32_t id, Node node);
            void add_or_assign_edge_SLOT(const std::int32_t from, const std::int32_t to, const std::string& type);
			void del_edge_SLOT(const std::int32_t from, const std::int32_t to,  const std::string &edge_tag);
			void del_node_SLOT(int id);
			void node_change_SLOT(int value,  int id, const std::string &type, QTreeWidgetItem* parent= nullptr);
			void category_change_SLOT(int value,  QTreeWidgetItem* parent= nullptr);
			void reload(QWidget *widget);

        private:
            std::shared_ptr<DSR::DSRGraph> G;
            std::map<std::string, QTreeWidgetItem*> types_map;
			std::map<int, QTreeWidgetItem*> tree_map;
			std::map<int, std::map<std::string, QTreeWidgetItem*>> attributes_map;
			void createGraph();
			void create_attribute_widgets(QTreeWidgetItem* parent, Node* node);
			void create_attribute_widget(QTreeWidgetItem* parent, Node* node, std::string key, Attribute value);
			void update_attribute_widgets(Node* node);

        
		signals:
			void node_check_state_changed(int newValue, int id, const std::string &type,  QTreeWidgetItem * item);
    };
};
#endif

