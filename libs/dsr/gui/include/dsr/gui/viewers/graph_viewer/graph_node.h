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
#include <utility>
#include <cppitertools/zip.hpp>

#include <dsr/api/dsr_api.h>
#include "graph_edge.h"
#include <dsr/gui/dsr_gui.h>

class GraphEdge;
class QGraphicsSceneMouseEvent;

class DoLaserStuff : public QGraphicsView
{
  Q_OBJECT
  public:
    DoLaserStuff(std::shared_ptr<DSR::DSRGraph> graph_, std::uint64_t node_id_) : graph(std::move(graph_)), node_id(node_id_)
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

class DoRGBDStuff : public QWidget
{
  Q_OBJECT
  QLabel rgbd_label, depth_label;
  QMenuBar *mainMenu;
  QAction *show_rgb;
  QAction *show_depth;
  std::chrono::system_clock::time_point last_update_rgb, last_update_d ;

  public:
    DoRGBDStuff(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(std::move(graph_)), node_id(node_id_)
    {
      //cam = graph->get_camera_api(graph->get_nodes_by_type("rgbd").at(0));
      setWindowTitle(QString::fromStdString(graph->get_agent_name()) + "-RGBD");
      //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoRGBDStuff::drawRGBDSLOT);
      QObject::connect(graph.get(), &DSR::DSRGraph::update_node_attr_signal, this, &DoRGBDStuff::drawRGBDSLOT);
      QHBoxLayout *layout = new QHBoxLayout();
      layout->addWidget(&rgbd_label);
      layout->addWidget(&depth_label);
      setLayout(layout);
      //MenuBar
      mainMenu = new QMenuBar(this);
      QMenu *viewMenu = mainMenu->addMenu(DoRGBDStuff::tr("&View"));
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

    void closeEvent (QCloseEvent *event) override
    {
        disconnect(graph.get(), nullptr, this, nullptr);
        //graph.reset();
        //cam.reset();
    };

  public slots:
    void drawRGBDSLOT( uint64_t id, const std::vector<std::string> &type)
    {
      if( static_cast<uint64_t>(id) != node_id) return;
      bool rgb = std::any_of(type.begin(), type.end(), [](auto& e){ return e == cam_rgb_att::attr_name;});
      bool d = std::any_of(type.begin(), type.end(), [](auto& e){ return e == cam_depth_att::attr_name;});

      auto now = std::chrono::system_clock::now();
      bool update_rgb = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_rgb)  > std::chrono::milliseconds(20);
      bool update_d = d = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_d)  > std::chrono::milliseconds(20);


      if ( (rgb and update_rgb) or (d and update_d) )
      {
          std::optional<Node> n = (show_rgb->isChecked() or show_depth->isChecked()) ?  graph->get_node(id) :  std::nullopt;
          if (n.has_value()) {
              Node node = n.value();
              //auto t = get_unix_timestamp();
              //std::cout << "[DRAW IMG] " << timestamp << ", " << t << ": " << static_cast<double>(t - timestamp) / 1000000 << std::endl;
              if (cam == nullptr) {
                  if (graph->get_attrib_by_name<cam_rgb_focalx_att>(node).has_value())
                      cam = graph->get_camera_api(node);
                  else return;
              }
              //rgb
              if (rgb and update_rgb) {
                  if (show_rgb->isChecked()) {
                      const auto rgb_data = cam->get_rgb_image();
                      const auto rgb_width = graph->get_attrib_by_name<cam_rgb_width_att>(node);
                      const auto rgb_height = graph->get_attrib_by_name<cam_rgb_height_att>(node);

                      if (rgb_data.has_value() and rgb_width.has_value() and rgb_height.has_value()) {
                          const std::vector<uint8_t> &img = rgb_data.value();//.get();
                          auto pix = QPixmap::fromImage(
                                  QImage(&img[0], rgb_width.value(), rgb_height.value(), QImage::Format_RGB888));
                          rgbd_label.setPixmap(pix);
                          last_update_rgb = now;
                      }
                  } else
                      rgbd_label.clear();
              }
              //depth
              if (d and update_d) {
                  if (show_depth->isChecked()) {
                      const auto depth_width = graph->get_attrib_by_name<cam_depth_width_att>(n.value());
                      const auto depth_height = graph->get_attrib_by_name<cam_depth_height_att>(n.value());
                      const std::optional<std::vector<std::uint8_t>> gray_scale = cam->get_depth_as_gray_image();
                      if (gray_scale.has_value() and depth_width.has_value() and depth_height.has_value()) {
                          depth_label.setPixmap(QPixmap::fromImage(
                                  QImage(&gray_scale.value()[0], depth_width.value(), depth_height.value(),
                                         QImage::Format_Indexed8)));
                          last_update_d = now;
                      }
                  } else {
                      depth_label.clear();
                  }
              }
              this->adjustSize();
              show();
          } else {
              rgbd_label.clear();
              depth_label.clear();
          }
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
    DoTableStuff(std::shared_ptr<DSR::DSRGraph> graph_, DSR::IDType node_id_) : graph(std::move(graph_)), node_id(node_id_)
    {
      qRegisterMetaType<std::int32_t>("std::int32_t");
      qRegisterMetaType<std::uint32_t>("std::uint32_t");
      qRegisterMetaType<std::uint64_t>("std::uint64_t");
      qRegisterMetaType<uint64_t>("uint64_t");
      qRegisterMetaType<std::string>("std::string");
      qRegisterMetaType<std::map<std::string, Attribute>>("Attribs");

      //setWindowFlags(Qt::Widget | Qt::FramelessWindowHint);
      std::optional<Node> n = graph->get_node(node_id_);
      if (n.has_value()) 
      {
          setWindowTitle("Node " + QString::fromStdString(n.value().type()) + " [" + QString::number(node_id) + "]");
          setColumnCount(2);
          std::map<std::string, DSR::Attribute>& attribs = n.value().attrs();
          // show id type and name as attributes
          attribs["ID"] = Attribute(ValType(node_id), 0, 0);
          attribs["type"] = Attribute(ValType(n.value().type()), 0, 0);
          attribs["name"] = Attribute(ValType(n.value().name()), 0, 0);
          setHorizontalHeaderLabels(QStringList{"Key", "Value"});
          for (auto &&[k, v] : attribs) {
                //TODO: check value range and attributes that could be editable
                insert_attribute(k, std::move(v));
          }
          horizontalHeader()->setStretchLastSection(true);
          resize_widget();
          //TODO: comprobar QObject::connect(graph.get(), &DSR::DSRGraph::update_attrs_signal, this, &DoTableStuff::drawSLOT);
          //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoTableStuff::update_node_slot);
          //QObject::connect(graph.get(), &DSR::DSRGraph::update_node_signal, this, &DoTableStuff::update_node_slot);
          QObject::connect(graph.get(), &DSR::DSRGraph::update_node_attr_signal, this, &DoTableStuff::update_node_attr_slot);
          show();
      }
    };

  public slots:
    void update_node_attr_slot(std::uint64_t node_id, const std::vector<std::string> &type)
    {
        if (node_id != this->node_id)
            return;
        std::optional<Node> n = graph->get_node(node_id);
        if (n.has_value()) {
            auto &attrs = n.value().attrs();
            for (const std::string &attrib_name :type )
            {
                auto value = attrs.find(attrib_name);
                if (value != attrs.end()) {
                    auto &&av = value->second;
                    if (widget_map.count(attrib_name)) {
                        update_attribute_value(attrib_name, std::move(av));
                    } else {
                        insert_attribute(attrib_name, std::move(av));
                    }

                }
            }
        }
    }
    void update_node_slot(std::uint64_t node_id, const std::string &type)
    {
        if (node_id != this->node_id)
            return;
        std::optional<Node> n = graph->get_node(node_id);
        if (n.has_value())
        {
            for (auto &&[k, v] : n.value().attrs()) {
                if(widget_map.count( k )) {
                    update_attribute_value(k, std::move(v));
                }
                else{
                    insert_attribute(k, std::move(v));
                }
            }
        }
    };
    void save_attribute_slot(const std::string &attrib_name)
    {
        std::cout<<"SAVE"<<attrib_name<<std::endl;
    }
    void resizeEvent(QResizeEvent* event) override
    {
      const auto &columns = columnCount();
      for(auto &&index : iter::range(columns))
          setColumnWidth(index, (width()-verticalHeader()->width()-4)/columns);
    }

    void closeEvent (QCloseEvent *event) override
    {
        disconnect(graph.get(), nullptr, this, nullptr);
        //graph.reset();
    };
  private:
    std::shared_ptr<DSR::DSRGraph> graph;
    std::uint64_t node_id;
    std::map<std::string, QWidget*> widget_map;
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
    void update_attribute_value(const std::string &k, DSR::Attribute &&v)
    {
        widget_map[k]->blockSignals(true);
        switch (v.selected()) {
            case 0: {
                qobject_cast<QLineEdit *>(widget_map[k])->setText(QString::fromStdString(v.str()));
                break;
            }
            case 1: {
                qobject_cast<QSpinBox *>(widget_map[k])->setValue(v.dec());
                break;
            }
            case 2: {
                qobject_cast<QDoubleSpinBox *>(widget_map[k])->setValue(std::round(static_cast<double>(v.fl()) * 1000000) / 1000000);
                break;
            }
            case 3: {
                //auto * widget = qobject_cast<QWidget*>(widget_map[k]);
                //auto * layout = widget->layout();
                //Esto no esta bien. No se controlan nuevos elementos, ni borrados, etc.
                /*if (!v.float_vec().empty() and v.float_vec().size() <= 10) {
                    for (size_t i = 0 ; i < v.float_vec().size(); ++i) {
                        qobject_cast<QDoubleSpinBox *>(layout->itemAt(i)->widget())->setValue(v.float_vec()[i]);
                    }
                }*/
                break;
            }
            case 4: {
                qobject_cast<QComboBox *>(widget_map[k])->setCurrentText(v.bl() ? "true" : "false");
                break;
            }
            case 5: {
                //auto * widget = qobject_cast<QWidget*>(widget_map[k]);
                //auto * layout = widget->layout();
                //Esto no esta bien. No se controlan nuevos elementos, ni borrados, etc.
                /*if (!v.byte_vec().empty() and v.byte_vec().size() <= 10) {
                    for (size_t i = 0 ; i < v.byte_vec().size(); ++i) {
                        qobject_cast<QSpinBox *>(layout->itemAt(i)->widget())->setValue(v.byte_vec()[i]);
                    }
                }*/
                break;
            }
            case 6:{
                qobject_cast<QSpinBox *>(widget_map[k])->setValue((int)v.uint());
                break;
            }
            case 7:{
                qobject_cast<QLineEdit *>(widget_map[k])->setText(QString::fromStdString(std::to_string(v.uint64())));
                break;
            }
            case 8:{
                qobject_cast<QDoubleSpinBox *>(widget_map[k])->setValue(std::round(v.dob()));
                break;
            }
        }
        widget_map[k]->blockSignals(false);
    }
    void insert_attribute(const std::string &k, DSR::Attribute &&v)
    {
        bool inserted = true;

        int rc = rowCount();
        insertRow( rc );

        switch (v.selected()) {
            case 0: {
                QLineEdit *ledit = new QLineEdit(QString::fromStdString(v.str()));
                setCellWidget(rc, 1, ledit);
                widget_map[k] = ledit;
                connect(ledit, &QLineEdit::textChanged, this, [this, k](const QString& text){
                    std::optional<Node> n = graph->get_node(node_id);
                    graph->runtime_checked_update_attrib_by_name(n.value(), k, text.toStdString());
                });
                break;
            }
            case 1:
            {
                QSpinBox *spin = new QSpinBox();
                spin->setMinimum(std::numeric_limits<int>::min());
                spin->setMaximum(std::numeric_limits<int>::max());
                spin->setValue(v.dec());
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, k](int value){
                    std::optional<Node> n = graph->get_node(node_id);
                    graph->runtime_checked_update_attrib_by_name(n.value(), k, value);
                });
                break;
            }
            case 2: {
                QDoubleSpinBox *spin = new QDoubleSpinBox();
                spin->setMinimum(-10000);
                spin->setMaximum(10000);
                spin->setValue(std::round(static_cast<double>(v.fl()) * 1000000) / 1000000);
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, k](double value){
                     std::optional<Node> n = graph->get_node(node_id);
                     graph->runtime_checked_update_attrib_by_name(n.value(), k, (float)value);
                });
                break;
            }
            case 3: {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                if (!v.float_vec().empty() and v.float_vec().size() <= 10) {
                    for (float i : v.float_vec()) {
                        QDoubleSpinBox *spin = new QDoubleSpinBox();
                        spin->setMinimum(-10000);
                        spin->setMaximum(10000);
                        spin->setValue(i);
                        layout->addWidget(spin);
                    }
                    setCellWidget(rc, 1, widget);
                    widget_map[k] = widget;
                } else {
                    inserted = false;
                }
                break;
            }
            case 4:{
                QComboBox *combo = new QComboBox();
                combo->addItem("true");
                combo->addItem("false");
                combo->setCurrentText(v.bl() ? "true" : "false");
                setCellWidget(rc, 1, combo);
                widget_map[k] = combo;
                connect(combo, &QComboBox::currentTextChanged, this, [this, k](const QString& value){
                    std::optional<Node> n = graph->get_node(node_id);
                    bool val = ( value == "true");
                    graph->runtime_checked_update_attrib_by_name(n.value(), k, val);
                });
                break;
            }
            case 5:
            {
                QWidget *widget = new QWidget();
                QHBoxLayout *layout = new QHBoxLayout;
                widget->setLayout(layout);
                if (!v.byte_vec().empty() and v.byte_vec().size() <= 10) {
                    for (unsigned char i : v.byte_vec()) {
                        QSpinBox *spin = new QSpinBox();
                        spin->setMinimum(0);
                        spin->setMaximum(255);
                        spin->setValue(i);
                        layout->addWidget(spin);
                    }
                    setCellWidget(rc, 1, widget);
                    widget_map[k] = widget;
                } else {
                    inserted = false;
                }
                break;
            }
            case 6:
            {
                QSpinBox *spin = new QSpinBox();
                spin->setMinimum(0);
                spin->setMaximum(std::numeric_limits<uint32_t>::max());
                spin->setValue((int)v.uint());
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, k](int value){
                    std::optional<Node> n = graph->get_node(node_id);
                    graph->runtime_checked_update_attrib_by_name(n.value(), k, (unsigned int)value);
                });
                break;
            }
            case 7:
            {
                QLineEdit *ledit = new QLineEdit(QString::fromStdString(std::to_string(v.uint64())));
                setCellWidget(rc, 1, ledit);
                widget_map[k] = ledit;
                break;
            }
            case 8:
            {
                QDoubleSpinBox *spin = new QDoubleSpinBox();
                spin->setMinimum(-10000);
                spin->setMaximum(10000);
                spin->setValue(v.dob());
                setCellWidget(rc, 1, spin);
                widget_map[k] = spin;
                connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this, k](double value){
                    std::optional<Node> n = graph->get_node(node_id);
                    graph->runtime_checked_update_attrib_by_name(n.value(), k, (double)value);
                });
                break;
            }
        }

        if (inserted)
        {
            auto *item =new QTableWidgetItem(QString::fromStdString(k));
            item->setFlags(Qt::ItemIsSelectable|Qt::ItemIsEnabled);
            setItem(rc, 0, item);

        } else {
            removeRow(rc);
        }

    }
};

class GraphNode : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT
    Q_PROPERTY(QColor node_color READ _node_color WRITE set_node_color)
	public:
    explicit GraphNode(std::shared_ptr<DSR::GraphViewer> graph_viewer_);

    //std::string name_in_graph;
    std::uint64_t id_in_graph;
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

    std::shared_ptr<DSR::GraphViewer> getGraphViewer() const { return dsr_to_graph_viewer;};
    void set_node_color(const QColor& c);
    void set_color(const std::string &plain);
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
    void update_node_attr_slot(std::uint64_t node_id, const std::vector<std::string> &type);
      
  private:
    QPointF newPos;
    QGraphicsSimpleTextItem *tag;
    QString dark_color = "darkyellow", plain_color = "yellow";
    std::string type;
    std::shared_ptr<DSR::GraphViewer> dsr_to_graph_viewer;
    QBrush node_brush;
	QPropertyAnimation* animation;
    QMenu *contextMenu = nullptr;
    std::unique_ptr<QWidget> do_stuff;
};
Q_DECLARE_METATYPE(DSR::Node);
#endif // GRAPHNODE_H
