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

#ifndef GRAPHNODE_H
#define GRAPHNODE_H

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
#include <cppitertools/zip.hpp>

#include "../../../api/dsr_api.h"
#include "graph_edge.h"
#include "../../dsr_gui.h"

class GraphEdge;
class QGraphicsSceneMouseEvent;

class DoLaserStuff : public QGraphicsView
{
  Q_OBJECT
  public:
    DoLaserStuff(std::shared_ptr<DSR::DSRGraph> graph_, std::int32_t node_id_) : graph(graph_), node_id(node_id_)
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
      //QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &DoLaserStuff::drawLaserSLOT);
      QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoLaserStuff::drawLaserSLOT);
      show();
    };

    void closeEvent (QCloseEvent *event) override 
    {
      disconnect(graph.get(), 0, this, 0);
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
      catch(const std::exception &e){ std::cout << "Node " << node_id << " problem. " << e.what() << std::endl;};
    };
  private:
    QGraphicsScene scene;
    std::shared_ptr<DSR::DSRGraph> graph;
    std::int32_t node_id;
};

class DoRGBDStuff : public QWidget
{
  Q_OBJECT
  QLabel rgbd_label, depth_label;
  QMenuBar *mainMenu;
  QAction *show_rgb;
  QAction *show_depth;
  
  public:
    DoRGBDStuff(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(graph_), node_id(node_id_)
    {
      //cam = graph->get_camera_api(graph->get_nodes_by_type("rgbd").at(0));

      setWindowTitle(QString::fromStdString(graph->get_agent_name()) + "-RGBD");
      QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoRGBDStuff::drawRGBDSLOT);
      QHBoxLayout *layout = new QHBoxLayout();
      layout->addWidget(&rgbd_label);
      layout->addWidget(&depth_label);
      setLayout(layout);
      //MenuBar
      mainMenu = new QMenuBar(this);
      QMenu *viewMenu = mainMenu->addMenu(this->tr("&View"));
      show_rgb = new QAction("RGB", this);
      show_rgb->setStatusTip(tr("Show RGB image"));
      show_rgb->setCheckable(true);
      show_rgb->setChecked(true);
      viewMenu->addAction(show_rgb);
      show_depth = new QAction("Depth", this);
      show_depth->setStatusTip(tr("Show DEPTH image"));
      show_depth->setCheckable(true);
      show_depth->setChecked(true);
      viewMenu->addAction(show_depth);
      show();
    };

    void closeEvent (QCloseEvent *event)
    {
      disconnect(graph.get(), 0, this, 0);
    };

  public slots:
    void drawRGBDSLOT( int id, const std::string &type )
    {
      if( static_cast<uint32_t>(id) != node_id) return;

      std::optional<Node> n = graph->get_node(id);
      if (n.has_value()) {
          Node node = n.value();
          if(cam == nullptr) {
              if(graph->get_attrib_by_name<cam_rgb_focalx_att>(node).has_value())
                cam = graph->get_camera_api(node);
              else return;
          }
          //rgb
          if (show_rgb->isChecked()) {
              const auto rgb_data = cam->get_rgb_image();
              const auto rgb_width = graph->get_attrib_by_name<cam_rgb_width_att>(node);
              const auto rgb_height = graph->get_attrib_by_name<cam_rgb_height_att>(node);

              if (rgb_data.has_value() and rgb_width.has_value() and rgb_height.has_value())
              {
                  const std::vector<uint8_t> &img = rgb_data.value().get();
                  auto pix = QPixmap::fromImage(
                          QImage(&img[0], rgb_width.value(), rgb_height.value(), QImage::Format_RGB888));
                  rgbd_label.setPixmap(pix);
              }
          } else
                rgbd_label.clear();
          //depth              
          if (show_depth->isChecked())
          {
            const auto depth_width = graph->get_attrib_by_name<cam_depth_width_att>(n.value());
            const auto depth_height = graph->get_attrib_by_name<cam_depth_height_att>(n.value());
            const std::optional<std::vector<std::uint8_t>> gray_scale = cam->get_depth_as_gray_image();
            if (gray_scale.has_value() and depth_width.has_value() and depth_height.has_value())
                depth_label.setPixmap(QPixmap::fromImage(QImage(&gray_scale.value()[0], depth_width.value(), depth_height.value(), QImage::Format_Indexed8)));
          }
          else{
            depth_label.clear();
          }
          this->adjustSize();
          show();
      }
    };

  private:
    QLabel label;
    std::shared_ptr<DSR::DSRGraph> graph;
    std::unique_ptr<CameraAPI> cam;
    DSR::IDType node_id;
};      

class DoTableStuff : public  QTableWidget
{
  Q_OBJECT
  public:
    DoTableStuff(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(graph_), node_id(node_id_)
    {
      qRegisterMetaType<std::int32_t>("std::int32_t");
      qRegisterMetaType<std::uint32_t>("std::uint32_t");
      qRegisterMetaType<std::string>("std::string");
      qRegisterMetaType<std::map<std::string, Attribute>>("Attribs");

      //setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
      std::optional<Node> n = graph->get_node(node_id_);
      if (n.has_value()) 
      {
          setWindowTitle("Node " + QString::fromStdString(n.value().type()) + " [" + QString::number(node_id) + "]");
          setColumnCount(2);
          std::map<std::string, DSR::Attribute> attribs;
          attribs = n.value().attrs();
          // show id type and name as attributes
          attribs["ID"] = Attribute(ValType(node_id), 0, 0);
          attribs["type"] = Attribute(ValType(n.value().type()), 0, 0);
          attribs["name"] = Attribute(ValType(n.value().name()), 0, 0);
          setRowCount(attribs.size());
          setHorizontalHeaderLabels(QStringList{"Key", "Value"});
          int i = 0;
          for (auto &[k, v] : attribs)
          {
              setItem(i, 0, new QTableWidgetItem(QString::fromStdString(k)));
              switch (v.selected()) {
                  case 0:
                      setItem(i, 1, new QTableWidgetItem(QString::fromStdString(v.str())));
                      break;
                  case 1:
                      setItem(i, 1, new QTableWidgetItem(
                              QString::fromStdString(std::get<1>(graph_->nativetype_to_string(v.dec())))));
                      break;
                  case 2:
                      setItem(i, 1, new QTableWidgetItem(
                              QString::fromStdString(std::get<1>(graph_->nativetype_to_string(v.fl())))));
                      break;
                  case 3:
                      setItem(i, 1, new QTableWidgetItem(QString::fromStdString(
                              std::get<1>(graph_->nativetype_to_string(v.float_vec())))));
                      break;
                  case 4:
                      setItem(i, 1, new QTableWidgetItem(QString(v.bl() ? "true" : "false")));
                      break;
                  case 5:
                      setItem(i, 1, new QTableWidgetItem(QString::fromStdString(
                              std::get<1>(graph_->nativetype_to_string(v.byte_vec())))));
                      break;
                  case 6:
                      setItem(i, 1, new QTableWidgetItem(
                              QString::fromStdString(std::get<1>(graph_->nativetype_to_string(v.uint())))));
                      break;
              }
              i++;
          }
          horizontalHeader()->setStretchLastSection(true);
          resize_widget();
          //TODO: comprobar QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &DoTableStuff::drawSLOT);
        
          show();
      }
    };

  public slots:
    void drawSLOT(const std::int32_t &id, const std::map<std::string,Attribute> &attribs)
    {
     // try {
          //std::cout << " Window " << this->window()->windowTitle().toStdString() << " id " << QString::number(id).toStdString() << " contains? " << this->window()->windowTitle().contains(QString::number(id)) << std::endl;
          if (this->window()->windowTitle().contains(QString::number(id))) {
              int i = 0;
              for (auto &[k, v] : attribs) {
                  setItem(i, 0, new QTableWidgetItem(QString::fromStdString(k)));
                  switch (v.selected()) {
                      case 0:
                          setItem(i, 1, new QTableWidgetItem(QString::fromStdString(v.str())));
                          break;
                      case 1:
                          setItem(i, 1, new QTableWidgetItem(
                                  QString::fromStdString(std::get<1>(graph->nativetype_to_string(v.dec())))));
                          break;
                      case 2:
                          setItem(i, 1, new QTableWidgetItem(
                                  QString::fromStdString(std::get<1>(graph->nativetype_to_string(v.fl())))));
                          break;
                      case 3:
                          setItem(i, 1, new QTableWidgetItem(
                                  QString::fromStdString(std::get<1>(graph->nativetype_to_string(v.float_vec())))));
                          break;
                      case 4:
                          setItem(i, 1, new QTableWidgetItem(QString(v.bl() ? "true" : "false")));
                          break;
                      case 5:
                          setItem(i, 1, new QTableWidgetItem(
                                  QString::fromStdString(std::get<1>(graph->nativetype_to_string(v.byte_vec())))));
                          break;
                      case 6:
                          setItem(i, 1, new QTableWidgetItem(
                                  QString::fromStdString(std::get<1>(graph->nativetype_to_string(v.uint())))));
                          break;
                  }
                  i++;
              }
          }
          resize_widget();
      //} catch (const std::exception &e) {
     //     std::cout << e.what() <<" Error  "<<__FUNCTION__<<":"<<__LINE__<<" "<<e.what()<< std::endl;
     // }
    }
    
    void resizeEvent(QResizeEvent* event)
    {
      const auto &columns = columnCount();
      for(auto &&index : iter::range(columns))
          setColumnWidth(index, (width()-verticalHeader()->width()-4)/columns);
    }

  private:
    std::shared_ptr<DSR::DSRGraph> graph;
    std::int32_t node_id;
    void resize_widget()
    {
        resizeRowsToContents();
        resizeColumnsToContents();
        int width = (this->model()->columnCount() - 1) + this->verticalHeader()->width() + 4;
        int height = (this->model()->rowCount() - 1) + this->horizontalHeader()->height() ;
        for(int column = 0; column < this->model()->columnCount(); column++)
            width = width + this->columnWidth(column);
        for(int row = 0; row < this->model()->rowCount(); row++)
            height = height + this->rowHeight(row);
        this->setMinimumWidth(width);
        this->setMinimumHeight(height);
    }
};

class GraphNode : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
    Q_PROPERTY(QColor node_color READ _node_color WRITE set_node_color)
	public:
    GraphNode(std::shared_ptr<DSR::GraphViewer> graph_viewer_);

    //std::string name_in_graph;
    std::int32_t id_in_graph;
    QList<GraphEdge *> edgeList;
    
    void addEdge(GraphEdge *edge);
    void deleteEdge(GraphEdge *edge);

    QList<GraphEdge *> edges() const;
    void calculateForces();
    bool advancePosition();
    void setTag(const std::string &tag_);
    std::string getTag() const { return tag->text().toStdString();};
    std::string getColor() const { return plain_color.toStdString(); };
    void setType(const std::string &type_);
    std::string getType() const { return type;};
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    void setColor(const std::string &plain);
    std::shared_ptr<DSR::GraphViewer> getGraphViewer() const { return dsr_to_graph_viewer;};
    void set_node_color(const QColor& c);
    QColor _node_color();
    void change_detected();
    
	protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
    //void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override { qDebug() << "move " << event->pos();};

  public slots:
    //void NodeAttrsChangedSLOT(const DSR::IDType &node, const DSR::Attribs&);
    void show_stuff_widget(const std::string &show_type="table");
      
	private:
    QPointF newPos;
    QGraphicsSimpleTextItem *tag;
    QString dark_color = "darkyellow", plain_color = "yellow";
    std::string type;
    std::shared_ptr<DSR::GraphViewer> dsr_to_graph_viewer;
    QBrush node_brush;
	QPropertyAnimation* animation;
    QMenu *contextMenu = nullptr;
    
};

#endif // GRAPHNODE_H
