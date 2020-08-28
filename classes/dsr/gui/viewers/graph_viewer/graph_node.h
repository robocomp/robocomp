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
            const auto lAngles = graph->get_attrib_by_name<angles_att>(n.value());
            const auto lDists = graph->get_attrib_by_name<dists_att>(n.value());
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
          //rgb
          if (show_rgb->isChecked()) {
              Node node = n.value();
              const auto rgb_data = graph->get_attrib_by_name<rgb_att>(node);
              const auto rgb_width = graph->get_attrib_by_name<width_att>(node);
              const auto rgb_height = graph->get_attrib_by_name<height_att>(node);

              if (rgb_data.has_value() and rgb_width.has_value() and rgb_height.has_value()) {
                  // if depth == 3
                  //image_rgb.create(height.value(), width.value(), CV_8UC3);
                  //QImage image((const unsigned char*)pixels, width, height, QImage::Format_RGB32);
                  //memcpy(image_rgb.data, &rgb_data.value()[0], width.value()*height.value()*sizeof(std::uint8_t)*3);
                  //imshow("RGB", image_rgb);
                  //cv::waitKey(1);
                  //label.setPixmap(QImage());

                  //QImage image((const unsigned char*)&rgb_data.value()[0], width.value(), height.value(), QImage::Format_RGB888);
                  //QImage image(width.value(), height.value(), QImage::Format_RGB888);
                  //memcpy(image.data, &rgb_data.value()[0], width.value()*height.value()*sizeof(std::uint8_t)*3);

                  const std::vector<uint8_t> &img = rgb_data.value();
                  auto pix = QPixmap::fromImage(
                          QImage(&img[0], rgb_width.value(), rgb_height.value(), QImage::Format_RGB888));
                  rgbd_label.setPixmap(pix);
              }
          } else
                rgbd_label.clear();
          //depth              
          if (show_depth->isChecked()) {
            const float factor = 255.f/4000.f; //define 4000 as max distance on grayscale conversion (255 value)
            const auto depth_width = graph->get_attrib_by_name<depth_width_att>(n.value());
            const auto depth_height = graph->get_attrib_by_name<depth_height_att>(n.value());
            const std::optional<std::vector<uint8_t>> depth_data = graph->get_attrib_by_name<img_depth_att>(n.value());
            if (depth_data.has_value() and depth_width.has_value() and depth_height.has_value()) {
                std::vector<uint8_t> gray_scale(depth_data.value().size()/4);
                float aux = 0.f;
                for (std::size_t i = 0; i < depth_data.value().size(); i+=4) {
                    //convert byte to float
                    if constexpr (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
                    {
                        *(unsigned int*)&aux = depth_data.value().at(i+3) << 24u | depth_data.value().at(i+2) << 16u | depth_data.value().at(i+1) << 8u | depth_data.value().at(i);
                    } else if constexpr(__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
                    {
                        *(unsigned int*)&aux = depth_data.value().at(i) << 24u | depth_data.value().at(i+1) << 16u | depth_data.value().at(i+2) << 8u | depth_data.value().at(i+3);
                    }
                    //convert float to grayscale => [(value - min(0)) / (max - min)] * 255
                    gray_scale.at(i/4) = aux * 255;
                }
                auto pix2 = QPixmap::fromImage(
                        QImage(&gray_scale[0], depth_width.value(), depth_height.value(), QImage::Format_Indexed8));
                depth_label.setPixmap(pix2);
            }
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
      qRegisterMetaType<map<string, Attribute>>("Attribs");

      //setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
      std::optional<Node> n = graph->get_node(node_id_);
      if (n.has_value()) 
      {
          setWindowTitle("Node " + QString::fromStdString(n.value().type()) + " [" + QString::number(node_id) + "]");
          setColumnCount(2);
          setRowCount(n.value().attrs().size());
          setHorizontalHeaderLabels(QStringList{"Key", "Value"});
          int i = 0;
          for (auto &[k, v] : n.value().attrs()) 
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
    void drawSLOT(const std::int32_t &id, const std::map<string,Attribute> &attribs)
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

	private:
    QPointF newPos;
    QGraphicsSimpleTextItem *tag;
    QString dark_color = "darkyellow", plain_color = "yellow";
    std::string type;
    std::shared_ptr<DSR::GraphViewer> dsr_to_graph_viewer;
    QBrush node_brush;
	QPropertyAnimation* animation;
};

#endif // GRAPHNODE_H
