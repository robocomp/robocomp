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

#ifndef DSRVIEWER_H
#define DSRVIEWER_H

#include <memory>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QListView>
#include <QMenuBar>
#include <dsr/api/dsr_api.h>
#include <typeinfo>
#include <QDockWidget>
#include <dsr/gui/viewers/osg_3d_viewer/osg_3d_viewer.h>
#include <dsr/gui/viewers/qscene_2d_viewer/qscene_2d_viewer.h>
#include <dsr/gui/viewers/graph_viewer/graph_viewer.h>
#include <dsr/gui/viewers/tree_viewer/tree_viewer.h>

#include <QFileDialog>


namespace DSR
{
//////////////////////////////////////////////////////////////////////////////////////////////
/// Drawing controller to display the graph in real-time using RTPS
//////////////////////////////////////////////////////////////////////////////////////////////

class DSRViewer : public QObject
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
    struct WidgetContainer
    {
        QString name;
        view type;
        QWidget * widget;
        QDockWidget* dock;
    };

    DSRViewer(QMainWindow *window, std::shared_ptr<DSR::DSRGraph> G, int options, view main = view::none);
    ~DSRViewer();
    QWidget* get_widget(view type);
    QWidget* get_widget(QString  name);
    void add_custom_widget_to_dock(QString name, QWidget* view);
    float get_external_hz() const;
    void set_external_hz(float external_hz);

protected:
    virtual void keyPressEvent(QKeyEvent *event);

private:
    QTimer *timer;
    QElapsedTimer alive_timer;
    std::shared_ptr<DSR::DSRGraph> G;
    QMainWindow *window;
    QMenu *viewMenu;
    QMenu *fileMenu;
    QMenu *forcesMenu;
//			std::shared_ptr<DSR::OSG3dViewer> dsr_to_osg_viewer;
//			std::shared_ptr<DSR::QScene2dViewer> dsr_to_graphicscene_viewer;
//			std::shared_ptr<DSR::GraphViewer> dsr_to_graph_viewer;
//			std::shared_ptr<DSR::DSRtoTreeViewer> dsr_to_tree_viewer;
    std::map<QString, QDockWidget *> docks;
    std::map<QString, WidgetContainer *> widgets;
    std::map<view, WidgetContainer *> widgets_by_type;
    QWidget * main_widget;
    QPoint * object_position;
    uint64_t object_id;
    float external_fps=-1;
    float external_hz=-1;

public slots:
    void saveGraphSLOT();
//			void toggleSimulationSLOT();
    void restart_app(bool);
    void switch_view(bool state, WidgetContainer* container);
    void compute();
    void qscene2d_object_position(int pos_x, int pos_y, uint64_t  node_id);
signals:
    void saveGraphSIGNAL();
    void closeWindowSIGNAL();
    void resetViewer(QWidget* widget);

private:
    void create_dock_and_menu(QString name,  QWidget *view);
    void initialize_views(int options, view main);
    void initialize_file_menu();
    QWidget * create_widget(view type);
};


};

Q_DECLARE_METATYPE(std::int32_t);
Q_DECLARE_METATYPE(std::string);

#endif // DSRVIEWER_H
