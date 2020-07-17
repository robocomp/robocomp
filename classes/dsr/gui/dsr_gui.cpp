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

#include "dsr_gui.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QDesktopWidget>
#include <QGLViewer/qglviewer.h>
#include <QApplication>
#include <QTableWidget>
#include <QStringList>
#include "viewers/graph_viewer/graph_node.h"
#include "viewers/graph_viewer/graph_edge.h"
#include "specificworker.h"
#include <unistd.h>

using namespace DSR;

std::string get_self_path()
{
	std::vector<char> buf(400);
	std::vector<char>::size_type len;

	do
	{
		buf.resize(buf.size() + 100);
		len = ::readlink("/proc/self/exe", &(buf[0]), buf.size());
	} while (buf.size() == len);

	if (len > 0)
	{
		buf[len] = '\0';
		return (std::string(&(buf[0])));
	}
	/* handle error */
	return "";
}

GraphViewer::GraphViewer(QMainWindow * widget, std::shared_ptr<DSR::DSRGraph> G_, int options, view main) : QObject()
{
	G = G_;
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
 	QRect availableGeometry(QApplication::desktop()->availableGeometry());
 	this->window = widget;
 	window->move((availableGeometry.width() - window->width()) / 2, (availableGeometry.height() - window->height()) / 2);
	
	// QSettings settings("RoboComp", "DSR");
    // settings.beginGroup("MainWindow");
    // 	graphicsView->resize(settings.value("size", QSize(400, 400)).toSize());
    // 	graphicsView->move(settings.value("pos", QPoint(200, 200)).toPoint());
    // settings.endGroup();
	// settings.beginGroup("QGraphicsView");
	// 	graphicsView->setTransform(settings.value("matrix", QTransform()).value<QTransform>());
	// settings.endGroup();

	//MenuBar
    viewMenu = window->menuBar()->addMenu(window->tr("&View"));
	auto actionsMenu = window->menuBar()->addMenu(window->tr("&Actions"));
	auto restart_action = actionsMenu->addAction("Restart");

	connect(restart_action, &QAction::triggered, this, [=] (bool)
	{
		qDebug()<<"TO RESTART";
		auto current_path = get_self_path();
		auto command = ("sleep 4 && "+current_path+"&");
		std::system(command.c_str());
//		QProcess a;
//		a.startDetached(command);
		qDebug()<<"TO RESTART2: "<<command.c_str();
		QTimer::singleShot(1000,QApplication::quit);
//		restart_app(true);
	});

	main_widget = nullptr;
	initialize_views(options, main);
}


GraphViewer::~GraphViewer()
{
	QSettings settings("RoboComp", "DSR");
    settings.beginGroup("MainWindow");
		settings.setValue("size", window->size());
		settings.setValue("pos", window->pos());
    settings.endGroup();
}


void GraphViewer::restart_app(bool)
{
//	qDebug()<<"TO RESTART";
//	auto executable_path = get_self_path();
//	std::system("/usr/bin/xclock&");
//	qDebug()<<command.c_str();
//
//
//	process->setWorkingDirectory("/home/robolab");
//	process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
//
//	process->start(command.c_str());
//	qDebug()<<process->readAllStandardOutput();
//	QTimer::singleShot(3000, QApplication::quit);
//	int pid = fork();
//	if (pid == 0)
//	{
//
//		auto command = std::string("sleep 1;/usr/bin/xclock");
//		qDebug()<<"TO RESTART"<<command.c_str();
//		// We are in the child process, execute the command
//		execl(command.c_str(), command.c_str(), nullptr);
//
//		// If execl returns, there was an error
//		std::cout << "Exec error: " << errno << ", " << strerror(errno) << '\n';
//
//		// Exit child process
//		exit(1);
//	}
//	else if (pid > 0)
//	{
//		// The parent process, do whatever is needed
//		// The parent process can even exit while the child process is running, since it's independent
//		QTimer::singleShot(3000, QApplication::quit);
//	}
//	else
//	{
//		// Error forking, still in parent process (there are no child process at this point)
//		std::cout << "Fork error: " << errno << ", " << strerror(errno) << '\n';
//	}
}

void GraphViewer::initialize_views(int options, view central){
	//Create docks view and main widget
	std::map<view,QString> valid_options{{view::graph,"Graph"}, {view::tree,"Tree"}, {view::osg,"3D"}, {view::scene, "2D"}};

	// creation of docks and mainwidget
	for (auto option: valid_options) {
		if(option.first == central and central != view::none)
		{
			auto viewer = create_widget(option.first);
			window->setCentralWidget(viewer);
			this->main_widget = viewer;
		}
		else if(options & option.first ) {
			auto viewer = create_widget(option.first);
			create_dock_and_menu(QString(option.second), viewer);
		}
	}

//	Tabification of curren docks
	QDockWidget * previous = nullptr;
	for(auto dock: docks) {
		if (previous)
			window->tabifyDockWidget(previous, dock.second);
		previous = dock.second;
	}

//	connection of tree to graph signals
	if(docks.count(QString("Tree"))==1) {
		if (this->main_widget) {
			auto graph_widget = qobject_cast<DSRtoGraphViewer*>(this->main_widget);
			if (graph_widget) {
				DSRtoTreeViewer* tree_widget = qobject_cast<DSRtoTreeViewer*>(docks["Tree"]->widget());
				GraphViewer::connect(
						tree_widget,
						&DSRtoTreeViewer::node_check_state_changed,
						graph_widget,
						[=](int value, int id, const std::string& type, QTreeWidgetItem*) {
							graph_widget->hide_show_node_SLOT(id, value==2); });
			}
		}
	}
	if((docks.size())>0 or central!=none)
	{
		window->show();
	}
	else {
		window->showMinimized();
	}

}


QWidget* GraphViewer::get_widget(view type)
{
	if(widgets.count(type)!=0)
		return widgets[type];
	return nullptr;
}

QWidget* GraphViewer::create_widget(view type){

	QWidget * widget_view = nullptr;
	switch(type) {
//		graph
		case view::graph:
			widget_view = new DSR::DSRtoGraphViewer(G);
			break;
//		3D
		case view::osg:
			widget_view = new DSR::DSRtoOSGViewer(G, 1, 1);
			break;
//		Tree
		case view::tree:
			widget_view = new DSR::DSRtoTreeViewer(G);
			break;
//		2D
		case view::scene:
			widget_view = new DSR::DSRtoGraphicsceneViewer(G);
			break;
		case view::none:
			break;
	}
	widgets[type] = widget_view;
	return widget_view;
}

void GraphViewer::create_dock_and_menu(QString name, QWidget* view){
//	TODO: Check if name exists in docks
	QDockWidget* dock_widget;
	if(this->docks.count(name)) {
		dock_widget = this->docks[name];
		window->removeDockWidget(dock_widget);
	}
	else{
		dock_widget = new QDockWidget(name);
		viewMenu->addAction(dock_widget->toggleViewAction());
		this->docks[name] = dock_widget;
	}
	dock_widget->setWidget(view);
	window->addDockWidget(Qt::RightDockWidgetArea, dock_widget);
}



////////////////////////////////////////
/// UI slots
////////////////////////////////////////
void GraphViewer::saveGraphSLOT()
{ 
	emit saveGraphSIGNAL(); 
}

//void GraphViewer::toggleSimulationSLOT()
//{
//	this->do_simulate = !do_simulate;
//	if(do_simulate)
//	   timerId = window->startTimer(1000 / 25);
//}



/////////////////////////
///// Qt Events
/////////////////////////

void GraphViewer::keyPressEvent(QKeyEvent* event) 
{
	if (event->key() == Qt::Key_Escape)
		emit closeWindowSIGNAL();
}

