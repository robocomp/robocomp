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
