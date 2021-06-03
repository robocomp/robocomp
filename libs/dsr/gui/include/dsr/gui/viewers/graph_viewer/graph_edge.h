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

#ifndef GRAPHEDGE_H
#define GRAPHEDGE_H

#include <QGraphicsItem>
#include <QContextMenuEvent>
#include <QTableWidget>
#include <memory>
#include <QPen>
#include <utility>
#include <cppitertools/zip.hpp>
#include <QLabel>
#include <dsr/gui/dsr_gui.h>
#include <qmat/QMatAll>
#include <QHeaderView>
#include <cppitertools/range.hpp>

using namespace DSR;

enum ItemFlags
{
    IF_FramelessSelection = 1,
    IF_DeleteAllowed = 2,
    IF_LastFlag = 4
};

enum ConnectionFlags	// extends ItemFlags
{
    CF_Start_Arrow		= IF_LastFlag,
    CF_End_Arrow		= IF_LastFlag << 2,
    CF_Mutual_Arrows	= CF_Start_Arrow | CF_End_Arrow		// start | end
};

class GraphEdge : public QObject, public QGraphicsLineItem, public std::enable_shared_from_this<GraphEdge>
{
Q_OBJECT
Q_PROPERTY(int edge_pen READ _edge_pen WRITE set_edge_pen)
protected:
//    QPointF m_controlPoint, m_controlPos;
//    QPainterPath m_shapeCachePath;
//    QPolygon arrow_polygon_cache;
    const int ARROW_SIZE = 6;
    const int NODE_DIAMETER = 20;
    const int NODE_RADIUS = NODE_DIAMETER/2;
private:
    GraphNode *source, *dest;
    qreal arrowSize;
    QGraphicsTextItem *tag;
    QColor color;
//    QGraphicsTextItem *rt_values = nullptr;
    QTableWidget *label = nullptr;
    int line_width;
    int m_bendFactor = 0;
    QPropertyAnimation* animation;
//    QPolygonF tag_polygon;
//    bool graphic_debug;
//    QRectF calculatedBoundingRect;
//    int bend_factor;
//    int m_itemFlags;
//    int m_internalStateFlags;

public:
    GraphEdge(GraphNode *sourceNode, GraphNode *destNode, const QString &edge_name);
    GraphNode *sourceNode() const;
    GraphNode *destNode() const;
    void adjust(GraphNode* node= nullptr, QPointF pos=QPointF());
//    int type() const override { return Type; }
//    QString getTag() const { return tag->text();};
    int _edge_pen();
    void set_edge_pen(const int with);
    void change_detected();
    void set_bend_factor(int bf);
//    bool isValid() const	{ return source != NULL && dest != NULL; }
//    bool isCircled() const	{ return isValid() && source == dest; }
    bool operator==(const GraphEdge& other){
        return ((source == other.source) and (dest == other.dest) and (tag->toPlainText() == other.tag->toPlainText()));
    };

protected:
    QRectF boundingRect() const override;
//    QRectF calculateBoundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    void keyPressEvent(QKeyEvent *event) override;
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;
    void update_edge_attr_slot(std::uint64_t from, std::uint64_t to, const std::vector<std::string>& att_name);
    bool eventFilter(QObject* object, QEvent* event) override;
private:
    void mouse_double_clicked();
    void draw_arc(QPainter* painter) const;
    void draw_arrows(QPainter* painter) const;
    void draw_line(QPainter* painter) const;





//private:
//    /*virtual*/ void drawArrow(QPainter *painter, const QStyleOptionGraphicsItem *option, bool first, const QLineF &direction) const;
//    /*virtual*/ void drawArrow(QPainter *painter, qreal shift, const QLineF &direction) const;
//    QLineF calculateArrowLine(const QPainterPath &path, bool first, const QLineF &direction) const;

};
//Q_DECLARE_METATYPE(DSR::Edge);
#endif // GRAPHEDGE_H
