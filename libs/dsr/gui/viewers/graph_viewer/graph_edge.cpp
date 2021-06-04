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

#include <dsr/gui/viewers/graph_viewer/graph_edge.h>
#include <dsr/gui/viewers/graph_viewer/graph_node.h>
#include <qmath.h>
#include <QPainter>
#include <QDebug>
#include <dsr/gui/dsr_gui.h>
#include <QGraphicsSceneMouseEvent>
#include <iostream>
#include <cppitertools/range.hpp>
#include <dsr/gui/viewers/graph_viewer/graph_colors.h>
#include <dsr/gui/viewers/graph_viewer/graph_edge_widget.h>
#include <dsr/gui/viewers/graph_viewer/graph_edge_rt_widget.h>
//#include <dsr/gui/viewers/graph_viewer/edge_colors.h>

GraphEdge::GraphEdge(GraphNode* sourceNode, GraphNode* destNode, const QString& edge_name)
        :QGraphicsLineItem(), arrowSize(10)
{
//    this->graphic_debug = true;
    source = dest = NULL;
    setZValue(-1);
    // non-movable but selectable
    auto flags = ItemIsSelectable | ItemSendsGeometryChanges | ItemUsesExtendedStyleOption;
    setFlags(flags);
    tag = new QGraphicsTextItem(edge_name, this);
    tag->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable);
    tag->installEventFilter(this);
//    tag->setCacheMode(DeviceCoordinateCache);
//    tag->setPen(Qt::NoPen);
    color = QString::fromStdString(GraphColors<DSR::Edge>()[edge_name.toStdString()]);
    animation = new QPropertyAnimation(this, "edge_pen");
    animation->setDuration(200);
    animation->setStartValue(4);
    animation->setEndValue(2);
    animation->setLoopCount(3);
//    bend_factor = 0;
    line_width = 2;

    // no selection frame
//    setItemFlag(IF_FramelessSelection);

    // accept hovers
    setAcceptHoverEvents(true);
    source = sourceNode;
    dest = destNode;
    source->addEdge(this);
    dest->addEdge(this);
    // label
//    m_itemFlags = CF_Mutual_Arrows;
    adjust();
    QObject::connect(source->getGraphViewer()->getGraph().get(), &DSR::DSRGraph::update_edge_attr_signal, this,
            &GraphEdge::update_edge_attr_slot);
}

GraphNode* GraphEdge::sourceNode() const
{
    return source;
}

GraphNode* GraphEdge::destNode() const
{
    return dest;
}

void GraphEdge::adjust(GraphNode* node, QPointF pos)
{
    if (!source || !dest)
        return;

    QLineF the_line(mapFromItem(source, 0, 0), mapFromItem(dest, 0, 0));
    qreal length = the_line.length();

    prepareGeometryChange();
    tag->setPos(line().center());
    if (length > qreal(GraphNode::DEFAULT_DIAMETER)) {
        QPointF edgeOffset((the_line.dx() * GraphNode::DEFAULT_RADIUS) / length, (the_line.dy() * GraphNode::DEFAULT_RADIUS) / length);
        QPointF sourcePoint = the_line.p1() + edgeOffset;
        QPointF destPoint = the_line.p2() - edgeOffset;
        setLine(QLineF(sourcePoint, destPoint));
    } else {
        setLine(the_line);
    }
//    if (length>qreal(20.)) {
//        QPointF edgeOffset((the_line.dx()*10)/length, (the_line.dy()*10)/length);
//        sourcePoint = the_line.p1()+edgeOffset;
//        destPoint = the_line.p2()-edgeOffset;
//    }
//    else {
//        sourcePoint = destPoint = the_line.p1();
//    }
}

//void GraphEdge::adjust(GraphNode* node, QPointF pos)
//{
//
//    if (!source || !dest)
//        return;
//
//    prepareGeometryChange();
//
//    // update line position
//    QPointF p1c = source->pos();
//    if (m_firstPortId.size() && source->getPort(m_firstPortId))
//        p1c = source->getPort(m_firstPortId)->scenePos();
//
//    QPointF p2c = dest->pos();
//    if (m_lastPortId.size() && dest->getPort(m_lastPortId))
//        p2c = dest->getPort(m_lastPortId)->scenePos();
//
//    QPointF p1 = source->getIntersectionPoint(QLineF(p1c, p2c), m_firstPortId);
//    QPointF p2 = dest->getIntersectionPoint(QLineF(p2c, p1c), m_lastPortId);
//
//    bool intersected = (!p1.isNull()) && (!p2.isNull());
//
//    QLineF l(p1, p2);
//    setLine(l);
//
//
//    // update shape path
//    m_shapeCachePath = QPainterPath();
//
//    double arrowSize = getVisibleWeight() + ARROW_SIZE;
//
//    // circled connection
//    if (isCircled())
//    {
//        int nodeDiameter = source->boundingRect().height();
//        double nr = nodeDiameter;
//        double r = nr + qAbs(m_bendFactor) * nr / 4;
//
//        // left up point
//        QPointF lp = p1c + QPointF(-r, -r);
//        QPointF p1 = source->getIntersectionPoint(QLineF(p1c, lp), m_firstPortId);
//
//        // right up point
//        QPointF rp = p2c + QPointF(r, -r);
//        QPointF p2 = dest->getIntersectionPoint(QLineF(p2c, rp), m_lastPortId);
//
//        // up point
//        m_controlPos = (p1c + p2c) / 2 + QPointF(0, -r * 2);
//        m_controlPoint = (lp + rp) / 2;
//
//        QLineF l(p1, p2);
//        setLine(l);
//
//        createCurvedPath(true, l, QLineF(p1c, p2c), p1, lp, rp, p2, arrowSize);
//    }
//    else // not circled
//    {
//        // center
//        m_controlPos = (p1c + p2c) / 2;
//
//        if (m_bendFactor == 0)
//        {
//            // shift line by arrows
//            auto len = l.length();
//            bool isArrow = (len > arrowSize * 2);
//
//            if (isArrow && (m_itemFlags & CF_Mutual_Arrows))
//            {
//                l = CUtils::extendLine(l,
//                        m_itemFlags & CF_Start_Arrow ? -arrowSize : 0,
//                        m_itemFlags & CF_End_Arrow ? arrowSize : 0);
//            }
//
//            m_shapeCachePath.moveTo(l.p1());
//            m_shapeCachePath.lineTo(l.p2());
//
//#if QT_VERSION < 0x050a00
//            m_controlPoint = (line().p1() + line().p2()) / 2;
//#else
//            m_controlPoint = line().center();
//#endif
//            auto fullLen = QLineF(p1c, p2c).length();
//            //qDebug() << len << fullLen;
//
//            // if no intersection or len == fullLen : drop the shape
//            if (!intersected || qAbs(len - fullLen) < 5)
//            {
//                m_shapeCachePath = QPainterPath();
//            }
//        }
//        else
//        {
//            QPointF t1 = m_controlPos;
//            float posFactor = qAbs(m_bendFactor);
//
//            bool bendDirection = (quint64(source) > quint64(dest));
//            if (m_bendFactor < 0)
//                bendDirection = !bendDirection;
//
//            QLineF f1(t1, p2c);
//            f1.setAngle(bendDirection ? f1.angle() + 90 : f1.angle() - 90);
//            f1.setLength(f1.length() * 0.2 * posFactor);
//
//            m_controlPos = f1.p2();
//            m_controlPoint = m_controlPos - (t1 - m_controlPos) * 0.33;
//
//            createCurvedPath(intersected, l, QLineF(p1c, p2c), p1, m_controlPoint, m_controlPoint, p2, arrowSize);
//        }
//    }
//
//    // rasterise the line stroke
//    QPainterPathStroker stroker;
//    stroker.setWidth(6);
//    m_selectionShapePath = stroker.createStroke(m_shapeCachePath);
//
//    //update();
//
//    // update text label
//    if (getScene() && getScene()->itemLabelsEnabled())
//    {
//        if (m_shapeCachePath.isEmpty())
//        {
//            m_labelItem->hide();
//        }
//        else
//        {
//            m_labelItem->show();
//
//            updateLabelPosition();
//            updateLabelDecoration();
//        }
//    }
//}

//QRectF GraphEdge::calculateBoundingRect() const
//{
//    int d = arrowSize;
//    // self returning edges
////    qDebug()<<__FUNCTION__ <<line().p1()<< line().p2();
//    if (qFuzzyCompare(line().length(), qreal(0.))) {
//        return QRectF(source->boundingRect().united(dest->boundingRect())).adjusted(-d, -d, d, d);
//    }
//    else {
////        qDebug()<<__FUNCTION__<<QRectF(line().p1(), line().p2())<<arrow_polygon_cache.boundingRect();
////        return QRectF(line().p1(), line().p2()).united(arrow_polygon_cache.boundingRect()).adjusted(-d, -d, d, d);
//        qDebug()<<__FUNCTION__<<QRectF(line().p1(), line().p2())<<arrow_polygon_cache.boundingRect();
//        return QRectF(line().p1(), line().p2())+=QMargins(d,d,d,d);//.adjusted(-NODE_RADIUS,-NODE_RADIUS,NODE_RADIUS,NODE_RADIUS).adjusted(-d, -d, d, d);
//    }
//}

QRectF GraphEdge::boundingRect() const
{
    if (!source || !dest)
        return QRectF();

    qreal penWidth = 1;
    qreal extra = (penWidth + arrowSize) / 2.0;

    return QRectF(line().p1(), QSizeF(line().p2().x() - line().p1().x(),
            line().p2().y() - line().p1().y()))//.united(source->boundingRect()).united(dest->boundingRect())
            .normalized()
            .adjusted(-extra, -extra, extra, extra);
//    qDebug()<<QGraphicsLineItem::boundingRect();
//    return QGraphicsLineItem::boundingRect();
//    if (this->graphic_debug) { return QRectF(-400, -400, 800, 800); }
//    else {
//        return calculateBoundingRect();
//    }
}


void GraphEdge::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget*)
{
    painter->save();
    if (!source || !dest)
        return;

    //if (source->collidesWithItem(dest))
    //    return;

//    draw_line(painter);
    draw_arrows(painter);
    draw_arc(painter);
    painter->restore();

    //    if (!source || !dest)
//        return;
//
//    // DEBUG
//    if (this->graphic_debug) {
//        //        auto fm = QFontMetrics(painter->font());
//        //        auto tag_rect = QRectF(-5,(-fm.height()-10)*0.5, fm.width(tag)+10, fm.height()+10);
//        //        tag_polygon = QTransform()
//        //                .translate(line().center().x(), line().center().y())
//        //                .rotate(-angle*180/M_PI)
//        //                .map(QPolygonF(tag_rect));
//        painter->setBrush(QBrush());
//        painter->setPen(QPen(Qt::red, 2, Qt::DashLine));
//
//        QRectF a = this->calculateBoundingRect();
//            painter->drawRect(a);
//        //        painter->drawEllipse(line().p1(), 5, 5);
//        //        painter->drawEllipse(line().p2(), 5, 5);
//        //        painter->drawPolygon(tag_polygon);
//        //        painter->setBrush(QBrush(Qt::black));
//        //        if (true) {
//        //            painter->setBrush(QBrush(QColor(128, 128, 255, 128)));
//        //            painter->setPen(QPen(Qt::black, 2, Qt::DashLine));
//        //            painter->drawPath(this->shape());
//        //        }
//    }
//
//
//    // self returning edges
//    painter->save();
//    if (qFuzzyCompare(line().length(), qreal(0.))) {
//        // Draw the line itself
//        qDebug()<<__FUNCTION__ <<"self line";
//        painter->setPen(QPen(color, line_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
//        QRectF rectangle(line().p1().x()-20, line().p1().y()-20, 20.0, 20.0);
//        this->calculatedBoundingRect = rectangle;
//        int startAngle = 35;
//        int spanAngle = 270*16;
//        painter->drawArc(rectangle, startAngle, spanAngle);
////            painter->setPen(QColor("coral"));
////            painter->drawText(rectangle.center(), tag);
//            double alpha = 0;
//            double r = 20/2.f;
//            painter->setBrush(color);
//            painter->setPen(QPen(color, line_width));
//            painter->drawPolygon(QPolygonF() << QPointF(r*cos(alpha) + rectangle.center().x(), r*sin(alpha) + rectangle.center().y())
//                                                                             << QPointF(r*cos(alpha) + rectangle.center().x()-3, r*sin(alpha) + rectangle.center().y()-2)
//                                                                             << QPointF(r*cos(alpha) + rectangle.center().x()+2, r*sin(alpha) + rectangle.center().y()-2));
//    }
//    else {
//        //check if there is another parallel edge
//        // Draw the line itself
//        qDebug() << __FUNCTION__ << "straight line";
//        qDebug() << __FUNCTION__ << "\t" << line().p1() << line().p2() << " " << line().length();
//        qDebug() << __FUNCTION__ << "\t " << boundingRect();
//        painter->setPen(QPen(color, line_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
//        painter->setBrush(QBrush());
////        painter->setPen(QPen(color, line_width, Qt::DashLine));
//        double angle = std::atan2(-line().dy(), line().dx());
//        painter->translate(line().center().x(), line().center().y());
//        painter->rotate(-angle*180/M_PI);
//        QRectF rectangle(-line().length()*0.5+20/2, -10, line().length()-20, 20);
//        painter->drawArc(rectangle, 0, 180*16);
////        painter->drawLine(line());
//
//
////        painter->setPen(QColor("coral"));
////        painter->drawText(rectangle.center(), tag);
//
//
//        // Draw the arrows
//        QVector<QPoint> points_vector;
//        points_vector.append(QPoint(line().length()*0.5-10, 0));
//        points_vector.append(QPoint(line().length()*0.5-10, 0)-QPoint(sin(M_PI/2)*arrowSize, cos(M_PI/2)*arrowSize));
//        points_vector.append(QPoint(line().length()*0.5-10, 0)-QPoint(sin(M_PI/4)*arrowSize, cos(M_PI/4)*arrowSize));
//        arrow_polygon_cache = QPolygon(points_vector);
//        painter->drawPolygon(arrow_polygon_cache);
//        painter->setPen(pen());
//
//
//
//
////    auto len = line().length();
////    bool isArrow = (len>ARROW_SIZE*2);
////
////    bool isDirect = (!isCircled() && (bend_factor==0));
////    if (isDirect) {// straight line()
////        // arrows
////        if (isArrow && m_itemFlags & CF_Start_Arrow) {
////            QLineF arrowLine = calculateArrowLine(m_shapeCachePath, true, QLineF(m_controlPos, line().p1()));
////            drawArrow(painter, option, true, arrowLine);
////        }
////
////        if (isArrow && m_itemFlags & CF_End_Arrow) {
////            QLineF arrowLine = calculateArrowLine(m_shapeCachePath, false, QLineF(m_controlPos, line().p2()));
////            drawArrow(painter, option, false, arrowLine);
////        }
////    }
//    }
//    painter->restore();
}
void GraphEdge::draw_line(QPainter* painter) const
{
    QPen myPen = this->pen();
    myPen.setColor(this->color);
    painter->setPen(myPen);
    painter->setBrush(this->color);

    painter->drawLine(this->line());
}
void GraphEdge::draw_arrows(QPainter* painter) const
{// Draw the arrows
    double angle = atan2(-this->line().dy(), this->line().dx());

//    QPointF sourceArrowP1 = this->line().p1() + QPointF(sin(angle + M_PI / 3) * this->ARROW_SIZE,
//            cos(angle + M_PI / 3) * this->ARROW_SIZE);
//    QPointF sourceArrowP2 = this->line().p1() + QPointF(sin(angle + M_PI - M_PI / 3) * this->ARROW_SIZE,
//            cos(angle + M_PI - M_PI / 3) * this->ARROW_SIZE);
    QPointF destArrowP1 = this->line().p2() + QPointF(sin(angle - M_PI / 3) * this->ARROW_SIZE,
            cos(angle - M_PI / 3) * this->ARROW_SIZE);
    QPointF destArrowP2 = this->line().p2() + QPointF(sin(angle - M_PI + M_PI / 3) * this->ARROW_SIZE,
            cos(angle - M_PI + M_PI / 3) * this->ARROW_SIZE);

    painter->setBrush(this->color);
    painter->setPen(this->color);
//    painter->drawPolygon(QPolygonF() << line().p1() << sourceArrowP1 << sourceArrowP2);
    painter->drawPolygon(QPolygonF() << this->line().p2() << destArrowP1 << destArrowP2);
}

void GraphEdge::draw_arc(QPainter* painter) const
{
    auto m_controlPos = (this->line().p1() + this->line().p2()) / 2;
    QPointF t1 = m_controlPos;
    float posFactor = qAbs(m_bendFactor);

    bool bendDirection = true;
    if (m_bendFactor < 0)
        bendDirection = !bendDirection;

    QLineF f1(t1, this->line().p2());
    f1.setAngle(bendDirection ? f1.angle() + 90 : f1.angle() - 90);
    f1.setLength(f1.length() * 0.2 * posFactor);

    m_controlPos = f1.p2();
    auto m_controlPoint = m_controlPos - (t1 - m_controlPos) * 0.33;

    auto path = QPainterPath();
    path.moveTo(this->line().p1());
    path.cubicTo(m_controlPoint, m_controlPoint, this->line().p2());
    auto r = tag->boundingRect();
    int w = r.width();
    int h = r.height();
    tag->setDefaultTextColor(this->color);
    tag->setPos(m_controlPoint.x() - w / 2, m_controlPoint.y() - h / 2);
    painter->setBrush(Qt::NoBrush);
    painter->setPen(this->color);
    painter->drawPath(path);
}

//void GraphEdge::drawArrow(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, bool first,
//        const QLineF& direction) const
//{
//    if ((first && source) or (!first && dest)) {
//        drawArrow(painter, 0, direction);
//    }
//}
//
//void GraphEdge::drawArrow(QPainter* painter, qreal /*shift*/, const QLineF& direction) const
//{
//    static QPolygonF arrowHead;
//    if (arrowHead.isEmpty())
//        arrowHead << QPointF(0, 0) << QPointF(-ARROW_SIZE/2, ARROW_SIZE) << QPointF(ARROW_SIZE/2, ARROW_SIZE)
//                  << QPointF(0, 0);
//
//    QPen oldPen = painter->pen();
//    painter->save();
//
//    painter->setPen(QPen(oldPen.color(), oldPen.widthF(), Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
//    painter->setBrush(oldPen.color());
//
//    static QLineF hl(0, 0, 0, 100);
//    qreal a = direction.angleTo(hl);
//
//    painter->translate(direction.p2());
//    painter->rotate(180+a);
//    painter->translate(QPointF(0, oldPen.widthF()));
//    painter->drawPolygon(arrowHead);
//
//    painter->restore();
//}
//
//QLineF GraphEdge::calculateArrowLine(const QPainterPath& path, bool first, const QLineF& direction) const
//{
//    //// optimization: disable during drag or pan
//    //Qt::MouseButtons buttons = QGuiApplication::mouseButtons();
//    //if ((buttons & Qt::LeftButton) || (buttons & Qt::RightButton))
//    //	return direction;
//
//    //// optimization: disable during zoom
//    //Qt::KeyboardModifiers keys = QGuiApplication::keyboardModifiers();
//    //if (keys & Qt::ControlModifier)
//    //	return direction;
//
//
//    if (first && source) {
//        qreal arrowStart = path.percentAtLength(ARROW_SIZE);
//        return QLineF(path.pointAtPercent(arrowStart), direction.p2());
//    }
//    else if (!first && dest) {
//        qreal len = path.length();
//        qreal arrowStart = path.percentAtLength(len-ARROW_SIZE);
//        return QLineF(path.pointAtPercent(arrowStart), direction.p2());
//    }
//
//    return direction;
//}

void GraphEdge::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event)
{
    if (event->button()==Qt::RightButton) {
        mouse_double_clicked();
    }
    QGraphicsLineItem::mouseDoubleClickEvent(event);
}

// For Tag Double click
bool GraphEdge::eventFilter(QObject* object, QEvent* event)
{
    if(object == this->tag)
    {
        if(event->type() == QEvent::GraphicsSceneMouseDoubleClick){
            auto mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);
            if (mouseEvent->button()==Qt::RightButton) {
                mouse_double_clicked();
            }
            return true;
        }
    }
    return false;
}

void GraphEdge::mouse_double_clicked()
{
    qDebug() << __FILE__ << " " << __FUNCTION__ << "Edge from " << this->source->id_in_graph << " to " << this->dest->id_in_graph
             << " tag: " << this->tag->toPlainText();
    static std::unique_ptr<QWidget> do_stuff;
    const auto graph = this->source->getGraphViewer()->getGraph();

    if (this->tag->toPlainText()=="RT" or this->tag->toPlainText()=="looking-at") {
        do_stuff = std::make_unique<GraphEdgeRTWidget>(graph, this->source->id_in_graph, this->dest->id_in_graph,
                this->tag->toPlainText().toStdString());
    }
    else {
        do_stuff = std::make_unique<GraphEdgeWidget>(graph, this->source->id_in_graph, this->dest->id_in_graph,
                this->tag->toPlainText().toStdString());
    }
    this->animation->start();
    this->update();
}

void GraphEdge::keyPressEvent(QKeyEvent* event)
{
    if (event->key()==Qt::Key_Escape) {
        if (label!=nullptr) {
            label->close();
            delete label;
            label = nullptr;
        }
    }
}

void GraphEdge::change_detected()
{
    animation->start();
}

int GraphEdge::_edge_pen()
{
    return this->line_width;
}

void GraphEdge::set_edge_pen(const int p)
{
    this->line_width = p;
    this->setPen(QPen(Qt::black, line_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
}

void GraphEdge::update_edge_attr_slot(std::uint64_t from, std::uint64_t to, const std::vector<std::string>& att_name)
{
    if ((from!=this->source->id_in_graph) or (to!=this->dest->id_in_graph))
        return;
    if (std::find(att_name.begin(), att_name.end(), "color")!=att_name.end()) {
        std::optional<Edge> edge = source->getGraphViewer()->getGraph()->get_edge(from, to, tag->toPlainText().toStdString());
        if (edge.has_value()) {
            auto& attrs = edge.value().attrs();
            auto value = attrs.find("color");
            if (value!=attrs.end()) {
                this->color = QColor(QString::fromStdString(value->second.str()));
            }
        }
    }
}
void GraphEdge::set_bend_factor(int bf)
{
    qDebug()<<__FUNCTION__ <<bf;
    m_bendFactor = bf;
}
