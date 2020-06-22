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

#include "CRDT_graphviewer.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QDesktopWidget>
#include <QGLViewer/qglviewer.h>
#include <QApplication>
#include <QTableWidget>
#include "CRDT_graphnode.h"
#include "CRDT_graphedge.h"
#include "specificworker.h"

using namespace DSR;

GraphViewer::GraphViewer(std::shared_ptr<CRDT::CRDTGraph> G_, std::list<View> options) :QMainWindow()
{
	G = G_;
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
 	QRect availableGeometry(QApplication::desktop()->availableGeometry());
 	this->move((availableGeometry.width() - width()) / 2, (availableGeometry.height() - height()) / 2);
	
	// QSettings settings("RoboComp", "DSR");
    // settings.beginGroup("MainWindow");
    // 	graphicsView->resize(settings.value("size", QSize(400, 400)).toSize());
    // 	graphicsView->move(settings.value("pos", QPoint(200, 200)).toPoint());
    // settings.endGroup();
	// settings.beginGroup("QGraphicsView");
	// 	graphicsView->setTransform(settings.value("matrix", QTransform()).value<QTransform>());
	// settings.endGroup();


	//MenuBar
    QMenu *viewMenu = this->menuBar()->addMenu(tr("&View"));

	//Create docks view
	//graph
	dsr_to_graph_viewer = std::make_unique<DSR::DSRtoGraphViewer>(G);	
	this->setCentralWidget(dsr_to_graph_viewer.get());

	//3D
	QDockWidget *osg_widget = new QDockWidget("3D");
	dsr_to_osg_viewer = std::make_unique<DSR::DSRtoOSGViewer>(G, 1, 1);
	osg_widget->setWidget(dsr_to_osg_viewer.get());
	this->addDockWidget(Qt::RightDockWidgetArea, osg_widget);
	QAction *action3D = viewMenu->addAction("&3D");
    action3D->setCheckable(true);
    action3D->setChecked(true);
	connect(action3D, SIGNAL(triggered(bool)), osg_widget, SLOT(setVisible(bool)));

	//Tree
	QDockWidget *tree_widget = new QDockWidget("Tree");
	dsr_to_tree_viewer = std::make_unique<DSR::DSRtoTreeViewer>(G);
	tree_widget->setWidget(dsr_to_tree_viewer.get());
	this->addDockWidget(Qt::RightDockWidgetArea, tree_widget);
	this->tabifyDockWidget(tree_widget, osg_widget);
	QAction *actionTree = viewMenu->addAction("&Tree");
    actionTree->setCheckable(true);
    actionTree->setChecked(true);
	connect(actionTree, SIGNAL(triggered(bool)), tree_widget, SLOT(setVisible(bool)));

	//2D
	QDockWidget *scene_widget = new QDockWidget("2D");
	dsr_to_graphicscene_viewer = std::make_unique<DSR::DSRtoGraphicsceneViewer>(G, 1, 1);
	scene_widget->setWidget(dsr_to_graphicscene_viewer.get());
	this->addDockWidget(Qt::RightDockWidgetArea, scene_widget);
	this->tabifyDockWidget(tree_widget, scene_widget);
	QAction *action2D = viewMenu->addAction("&2D");
    action2D->setCheckable(true);
    action2D->setChecked(true);
	connect(action2D, SIGNAL(triggered(bool)), scene_widget, SLOT(setVisible(bool)));



/*	for(auto option: options)
	{
		if(option == View::Scene)
		 	dsr_to_graph_viewer = std::make_unique<DSR::DSRtoGraphViewer>(G, graphicsView);

		// //dsr_to_tree_viewer = std::make_unique<DSR::DSRtoTreeViewer>(G, treeWidget);
		if(option == View::OSG)
		 	dsr_to_osg_viewer = std::make_unique<DSR::DSRtoOSGViewer>(G, 1, 1, openGLWidget);
		// if(option == View::Graph)
		// 	dsr_to_graphicscene_viewer = std::make_unique<DSR::DSRtoGraphicsceneViewer>(G, 1, 1, graphicsView_2D);
	}*/

}

GraphViewer::~GraphViewer()
{
	QSettings settings("RoboComp", "DSR");
    settings.beginGroup("MainWindow");
		settings.setValue("size", size());
		settings.setValue("pos", pos());
    settings.endGroup();
}

////////////////////////////////////////
/// UI slots
////////////////////////////////////////
void GraphViewer::saveGraphSLOT()
{ 
	emit saveGraphSIGNAL(); 
}

void GraphViewer::toggleSimulationSLOT()
{
	this->do_simulate = !do_simulate;
	if(do_simulate)
	   timerId = startTimer(1000 / 25);
}

///////////////////////////////////////

void GraphViewer::itemMoved()
{
	//std::cout << "timerId " << timerId << std::endl;
	//if(do_simulate and timerId == 0)
    //if (timerId == 0)
    //   timerId = startTimer(1000 / 25);
}

void GraphViewer::timerEvent(QTimerEvent *event)
{
    // Q_UNUSED(event)

	// for( auto &[k,node] : gmap)
	// {
	// 	(void)k;
	//     node->calculateForces();
	// }
	// bool itemsMoved = false;
	
	// for( auto &[k,node] : gmap)
	// {
	// 	(void)k;
    //     if (node->advancePosition())
    //         itemsMoved = true;
    // }
	// if (!itemsMoved) 
	// {
    //     killTimer(timerId);
    //     timerId = 0;
    // }
}

/////////////////////////
///// Qt Events
/////////////////////////

void GraphViewer::keyPressEvent(QKeyEvent* event) 
{
	if (event->key() == Qt::Key_Escape)
		emit closeWindowSIGNAL();
}

