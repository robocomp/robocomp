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

#ifndef GRAPHVIEWER_H
#define GRAPHVIEWER_H

#include <memory>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QListView>
#include <QMenuBar>
#include "../api/dsr_api.h"
#include <typeinfo>
#include <QDockWidget>
#include "viewers/osg_3d_viewer/osg_3d_viewer.h"
#include "viewers/qscene_2d_viewer/qscene_2d_viewer.h"
#include "viewers/graph_viewer/graph_viewer.h"
#include "viewers/tree_viewer/tree_viewer.h"


namespace DSR
{
	//////////////////////////////////////////////////////////////////////////////////////////////77
	/// Drawing controller to display the graph in real-time using RTPS 
	//////////////////////////////////////////////////////////////////////////////////////////////77
	
	class GraphViewer : public QObject
	{
		Q_OBJECT
		public:
			enum view
			{
				none = -1,
				graph = (1 << 0),
				osg = (1 << 1),
				scene = (1 << 2),
				tree = (1 << 3),
			};
			GraphViewer(QMainWindow *window, std::shared_ptr<DSR::DSRGraph> G, int options, view main = view::none);
			~GraphViewer();
			void itemMoved();
			void createGraph();
			QWidget* get_widget(view type);

		protected:
			virtual void keyPressEvent(QKeyEvent *event);
		
		private:
			std::shared_ptr<DSR::DSRGraph> G;
			QMainWindow *window;
			QMenu *viewMenu;
//			std::shared_ptr<DSR::DSRtoOSGViewer> dsr_to_osg_viewer;
//			std::shared_ptr<DSR::DSRtoGraphicsceneViewer> dsr_to_graphicscene_viewer;
//			std::shared_ptr<DSR::DSRtoGraphViewer> dsr_to_graph_viewer;
//			std::shared_ptr<DSR::DSRtoTreeViewer> dsr_to_tree_viewer;
			std::map<QString, QDockWidget *> docks;
			std::map<view, QWidget *> widgets;
			QWidget * main_widget;

		public slots:
			void saveGraphSLOT();		
//			void toggleSimulationSLOT();
			void restart_app(bool);

		signals:
			void saveGraphSIGNAL();
			void closeWindowSIGNAL();
		private:
			void create_dock_and_menu(QString name,  QWidget *view);
			void initialize_views(int options, view main);
			QWidget * create_widget(view type);
	};
}

Q_DECLARE_METATYPE(std::int32_t);
Q_DECLARE_METATYPE(std::string);
//Q_DECLARE_METATYPE(DSR::AttribsMap);

#endif // GRAPHVIEWER_H
