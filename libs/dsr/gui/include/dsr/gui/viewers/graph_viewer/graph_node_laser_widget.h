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

#ifndef GRAPHNODELASERWIDGET_H
#define GRAPHNODELASERWIDGET_H

#include <QGraphicsEllipseItem>
#include <QTableWidget>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>
#include <QDebug>
#include <QDialog>
#include <QHeaderView>
#include <QLabel>
#include <utility>
#include <cppitertools/zip.hpp>

#include <dsr/api/dsr_api.h>
#include "graph_edge.h"
#include <dsr/gui/dsr_gui.h>


class GraphNodeLaserWidget : public QGraphicsView
{
  Q_OBJECT
  public:
    GraphNodeLaserWidget(std::shared_ptr<DSR::DSRGraph> graph_, std::uint64_t node_id_) : graph(std::move(graph_)), node_id(node_id_)
    {
      std::cout << __FUNCTION__ << std::endl;
      resize(400,400);
      setWindowTitle("Laser");
      scene.setSceneRect(-5000, -5000, 10000, 10000);
      setScene(&scene);
      setRenderHint(QPainter::Antialiasing);
      fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
      scale(1, -1);
      //drawLaserSLOT(node_id_, );
      //QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &GraphNodeLaserWidget::drawLaserSLOT);
      QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &GraphNodeLaserWidget::drawLaserSLOT);
      show();
    };


    void closeEvent (QCloseEvent *event) override 
    {
      disconnect(graph.get(), nullptr, this, nullptr);
      //graph.reset();
    };

  public slots:
    void drawLaserSLOT( int id, const std::string &type )
    {
      if( id != node_id )
        return;
      try
      {
        //std::cout << __FUNCTION__ <<"-> Node: "<<id<< std::endl;
        std::optional<Node> n = graph->get_node(id);
        if (n.has_value()) 
        {
            const auto lAngles = graph->get_attrib_by_name<laser_angles_att>(n.value());
            const auto lDists = graph->get_attrib_by_name<laser_dists_att>(n.value());
            if (lAngles.has_value() and lDists.has_value()) 
            {
                QPolygonF polig;
                polig << QPointF(0,150);
                for (const auto &[dist, angle] : iter::zip(lDists.value().get(), lAngles.value().get()))
                {
                    //std::cout << dist << "," << angle << std::endl;
                    polig << QPointF(dist * sin(angle), dist * cos(angle));
                }
                polig << QPointF(0,150);
                scene.clear();
                QPolygonF robot;
                robot << QPointF(-200, 200) << QPointF(-200, -200) << QPointF(200, -200) << QPointF(200, 200) << QPointF(0, 260);
                scene.addPolygon(polig, QPen(QColor("LightPink"), 8), QBrush(QColor("LightPink")));
                scene.addPolygon(robot, QPen(QColor("DarkGreen"), 8), QBrush(QColor("DarkGreen")));
            }
        }
      }
      catch(const std::exception &e){ std::cout << "Node " << node_id << " problem. " << e.what() << std::endl;}
    };
  private:
    QGraphicsScene scene;
    std::shared_ptr<DSR::DSRGraph> graph;
    std::int32_t node_id;
};

#endif // GRAPHNODELASERWIDGET_H
