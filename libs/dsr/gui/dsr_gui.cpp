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

#include <dsr/gui/dsr_gui.h>
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QDesktopWidget>
#include <QGLViewer/qglviewer.h>
#include <QApplication>
#include <QTableWidget>
#include <QStringList>
#include <QStatusBar>
#include <utility>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <dsr/gui/viewers/graph_viewer/graph_edge.h>
#include <unistd.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys/times.h"
#include "sys/vtimes.h"

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
	return "";
}

int parseLine(char* line)
{
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int get_memory_usage()
{
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

static clock_t lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;

void init(){
    FILE* file;
    struct tms timeSample;
//    char line[128];

    lastCPU = times(&timeSample);
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 1;
//    while(fgets(line, 128, file) != NULL){
//        if (strncmp(line, "processor", 9) == 0) numProcessors++;
//    }
    fclose(file);
}

double get_cpu_usage(){
    struct tms timeSample;
    clock_t now;
    double percent;

    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
        timeSample.tms_utime < lastUserCPU){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        percent = (timeSample.tms_stime - lastSysCPU) +
            (timeSample.tms_utime - lastUserCPU);
        percent /= (now - lastCPU);
        percent /= numProcessors;
        percent *= 100;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    return percent;
}


DSRViewer::DSRViewer(QMainWindow * widget, std::shared_ptr<DSR::DSRGraph> G_, int options, view main) : QObject()
{
	G = std::move(G_);
    qRegisterMetaType<std::int32_t>("std::int32_t");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<std::uint32_t>("std::uint32_t");
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");

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
    initialize_file_menu();
    viewMenu = window->menuBar()->addMenu(window->tr("&View"));
	forcesMenu = window->menuBar()->addMenu(window->tr("&Forces"));
	auto actionsMenu = window->menuBar()->addMenu(window->tr("&Actions"));
	auto restart_action = actionsMenu->addAction("Restart");

	//restart_action
	connect(restart_action, &QAction::triggered, this, [=] (bool)
	{
		qDebug()<<"TO RESTART";
		auto current_path = get_self_path();
		auto command = ("sleep 4 && "+current_path+"&");
		[[maybe_unused]] auto _ = std::system(command.c_str());
//		QProcess a;
//		a.startDetached(command);
		qDebug()<<"TO RESTART2: "<<command.c_str();
		QTimer::singleShot(1000,QApplication::quit);
//		restart_app(true);
	});

	main_widget = nullptr;
    object_position =  nullptr;
    initialize_views(options, main);
    timer = new QTimer();
    alive_timer.start();
    timer->start(500);
    init();  //intialize processor number
    connect(timer, SIGNAL(timeout()), this, SLOT(compute()));
}

void DSRViewer::initialize_file_menu()
{
    fileMenu = window->menuBar()->addMenu(window->tr("&File"));
    QMenu *file_submenu = fileMenu->addMenu("Save");
    QAction *save_action = new QAction("Save", this);
    file_submenu->addAction(save_action);
    QAction *rgbd = new QAction("RGBD", this);
    rgbd->setCheckable(true);
    rgbd->setChecked(false);
    file_submenu->addAction(rgbd);
    QAction *laser = new QAction("Laser", this);
    laser->setCheckable(true);
    laser->setChecked(false);
    file_submenu->addAction(laser);
	//save_action
    connect(save_action, &QAction::triggered, [this, rgbd, laser]() {
        auto file_name = QFileDialog::getSaveFileName(nullptr, tr("Save file"),
                                                      "/home/robocomp/robocomp/components/dsr-graph/etc",
                                                      tr("JSON Files (*.json)"), nullptr,
                                                      QFileDialog::Option::DontUseNativeDialog);

        std::vector<std::string> skip_content;
        if(not rgbd->isChecked()) skip_content.push_back("rgbd");
        if(not laser->isChecked()) skip_content.push_back("laser");
        G->write_to_json_file(file_name.toStdString(), skip_content);
        qDebug()<<"File saved";
    });

}

DSRViewer::~DSRViewer()
{
	QSettings settings("RoboComp", "DSR");
    settings.beginGroup("MainWindow");
		settings.setValue("size", window->size());
		settings.setValue("pos", window->pos());
    settings.endGroup();
}


void DSRViewer::restart_app(bool)
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

void DSRViewer::initialize_views(int options, view central){
	//Create docks view and main widget
	std::map<view,QString> valid_options{{view::graph,"Graph"}, {view::tree,"Tree"}, {view::osg,"3D"}, {view::scene, "2D"}};

	// creation of docks and mainwidget
	for (auto const& [type, name] : valid_options) {
		if(type == central and central != view::none)
		{
			auto viewer = create_widget(type);
			window->setCentralWidget(viewer);
			WidgetContainer * widget_c = new WidgetContainer();
			widget_c->widget = viewer;
			widget_c->name = name;
			widget_c->type = type;
			widgets[name] = widget_c;
			widgets_by_type[type]= widget_c;
			this->main_widget = viewer;
		}
		else if(options & type ) {
			auto viewer = create_widget(type);
			WidgetContainer * widget_c = new WidgetContainer();
			widget_c->widget = viewer;
			widget_c->name = name;
			widget_c->type = type;
			widgets[name] = widget_c;
			widgets_by_type[type]= widget_c;
			create_dock_and_menu(QString(name), viewer);
		}
	}
	if(widgets_by_type.count(view::graph))
	{
		QAction *new_action = new QAction("Animation", this);
		new_action->setStatusTip(tr("Toggle animation"));
		new_action->setCheckable(true);
		new_action->setChecked(false);
		forcesMenu->addAction(new_action);
		connect(new_action, &QAction::triggered, this, [this](bool state)
		{
			qobject_cast<GraphViewer*>(widgets_by_type[view::graph]->widget)->toggle_animation(state==true);
		});
	}

//	Tabification of current docks
	QDockWidget * previous = nullptr;
	for(auto dock: docks) {
		if (previous)
			window->tabifyDockWidget(previous, dock.second);
		previous = dock.second;
	}

//	connection of tree to graph signals
	if(docks.count(QString("Tree"))==1) {
		if (this->main_widget) {
            auto graph_widget = qobject_cast<GraphViewer*>(this->main_widget);
			if (graph_widget) {
				TreeViewer* tree_widget = qobject_cast<TreeViewer*>(docks["Tree"]->widget());
				DSRViewer::connect(
						tree_widget,
						&TreeViewer::node_check_state_changed,
						graph_widget,
						[=](int value, int id, const std::string& type, QTreeWidgetItem*) {
							graph_widget->hide_show_node_SLOT(id, value==2); });
			}
		}
	}
//  connection of 2d scene signals
    if(docks.count(QString("2D"))==1) {
        std::cout<<"connect 2D"<<std::endl;
        QScene2dViewer* qscene_widget = qobject_cast<QScene2dViewer*>(docks["2D"]->widget());
        DSRViewer::connect(qscene_widget, &QScene2dViewer::mouse_right_click, this, &DSRViewer::qscene2d_object_position);
    }
	if(widgets.size()<=0 and docks.size()<=0 and main_widget==nullptr)
	{
        qDebug()<<__FUNCTION__ <<"Minimize"<<widgets.size()<<docks.size()<<main_widget;
        window->showMinimized();
	}
	else {
        qDebug()<<__FUNCTION__ <<"Normal Show";
        window->show();
	}
}


QWidget* DSRViewer::get_widget(view type)
{
	if(widgets_by_type.count(type)!=0)
		return widgets_by_type[type]->widget;
	return nullptr;
}

QWidget* DSRViewer::get_widget(QString name)
{
	if(widgets.count(name)!=0)
		return widgets[name]->widget;
	return nullptr;
}

QWidget* DSRViewer::create_widget(view type){

	QWidget * widget_view = nullptr;
	switch(type) {
//		graph
		case view::graph:
			widget_view = new DSR::GraphViewer(G);
			break;
//		3D
		case view::osg:
			widget_view = new DSR::OSG3dViewer(G, 1, 1);
			break;
//		Tree
		case view::tree:
			widget_view = new DSR::TreeViewer(G);
			break;
//		2D
		case view::scene:
			widget_view = new DSR::QScene2dViewer(G);
			break;
		case view::none:
			break;
	}
	connect(this, SIGNAL(resetViewer(QWidget*)), widget_view, SLOT(reload(QWidget*)));
	return widget_view;
}

void DSRViewer::create_dock_and_menu(QString name, QWidget* view){
//	TODO: Check if name exists in docks
	QDockWidget* dock_widget;
	if(this->docks.count(name)) {
		dock_widget = this->docks[name];
		window->removeDockWidget(dock_widget);
	}
	else{
		dock_widget = new QDockWidget(name);
		QAction *new_action = new QAction(name, this);
		new_action->setStatusTip(tr("Create a new file"));
		new_action->setCheckable(true);
		new_action->setChecked(true);
		connect(new_action, &QAction::triggered, this, [this, name](bool state) {
			switch_view(state, widgets[name]);
		});
		viewMenu->addAction(new_action);
		this->docks[name] = dock_widget;
		widgets[name]->dock = dock_widget;
	}
	dock_widget->setWidget(view);
	dock_widget->setAllowedAreas(Qt::AllDockWidgetAreas);
	window->addDockWidget(Qt::RightDockWidgetArea, dock_widget);
	dock_widget->raise();
}

void DSRViewer::add_custom_widget_to_dock(QString name, QWidget* custom_view){
	WidgetContainer * widget_c = new WidgetContainer();
	widget_c->name = name;
	widget_c->type = view::none;
	widget_c->widget = custom_view;
	widgets[name] = widget_c;
	create_dock_and_menu(name, custom_view);
//	Tabification of current docks
	QDockWidget * previous = nullptr;
	for(auto& [dock_name, dock]: docks) {
		if (previous and previous!=dock) {
			window->tabifyDockWidget(previous, docks[name]);
			break;
		}
		previous = dock;
	}
	docks[name]->raise();
    window->setWindowState(window->windowState() & ~Qt::WindowMinimized | Qt::WindowActive);
}


////////////////////////////////////////
/// UI slots
////////////////////////////////////////
void DSRViewer::saveGraphSLOT()
{ 
	emit saveGraphSIGNAL(); 
}

void DSRViewer::switch_view(bool state, WidgetContainer* container)
{
	QWidget * widget = container->widget;
	QDockWidget * dock = container->dock;
	if(!state)
	{
		widget->blockSignals(true);
		dock->hide();
	}
	else{
		widget->blockSignals(false);
		emit resetViewer(widget);
		dock->show();
		dock->raise();
	}
}



//void DSRViewer::toggleSimulationSLOT()
//{
//	this->do_simulate = !do_simulate;
//	if(do_simulate)
//	   timerId = window->startTimer(1000 / 25);
//}


/////////////////////////
///// SLOTS
////////////////////////
void DSRViewer::compute()
{
    std::stringstream cpu_usage;
    cpu_usage << std::fixed << std::setprecision(2) << get_cpu_usage();

    std::stringstream memory_usage;
    memory_usage << std::fixed << std::setprecision(2) << get_memory_usage()/1024.f;
    std::string status = "ALIVE TIME: " + std::to_string(alive_timer.elapsed()/1000) + "s CPU: " + cpu_usage.str() + " Memory: " + memory_usage.str() + "MB ";
    if (object_position != nullptr)
    {
        status += " Object pose: (" + std::to_string(object_position->x()) + ", " + std::to_string(object_position->y()) + ") ID: " + std::to_string(object_id);
    }
    this->window->statusBar()->showMessage(QString::fromStdString(status)); 
}

void DSRViewer::qscene2d_object_position(int pos_x, int pos_y, uint64_t  node_id)
{
    if (object_position == nullptr)
        object_position = new QPoint();
    object_position->setX(pos_x);
    object_position->setY(pos_y);
    object_id = node_id;
}

/////////////////////////
///// Qt Events
/////////////////////////

void DSRViewer::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Escape)
		emit closeWindowSIGNAL();
}

