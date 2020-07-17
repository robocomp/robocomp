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

#include "graph_node.h"


GraphNode::GraphNode(std::shared_ptr<DSR::DSRtoGraphViewer> graph_viewer_): QGraphicsEllipseItem(0,0,30,30), dsr_to_graph_viewer(graph_viewer_)
{
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
    setZValue(-1);
    node_brush.setStyle(Qt::SolidPattern);
	animation = new QPropertyAnimation(this, "node_color");
	animation->setDuration(200);
	animation->setStartValue(plain_color);
	animation->setEndValue(dark_color);
	animation->setLoopCount(3);
}

void GraphNode::setTag(const std::string &tag_)
{
    QString c = QString::fromStdString(tag_);
	tag = new QGraphicsSimpleTextItem(c, this);
	tag->setX(20);	
	tag->setY(-10);
}

void GraphNode::setType(const std::string &type_)
{
    type = type_;
}

void GraphNode::setColor(const std::string &plain)
{
    QString c = QString::fromStdString(plain);
	plain_color = c;
	dark_color = "dark" + c;
    animation->setStartValue(QColor("green").lighter());
    animation->setEndValue(QColor(c));
    node_brush.setColor(QColor(c));

}

void GraphNode::addEdge(GraphEdge *edge)
{
    edgeList << edge;
    edge->adjust();
}

QList<GraphEdge *> GraphNode::edges() const
{
    return edgeList;
}

void GraphNode::calculateForces()
{
    if (!scene() || scene()->mouseGrabberItem() == this) 
	{
        newPos = pos();
        return;
    }

    // Sum up all forces pushing this item away
    qreal xvel = 0;
    qreal yvel = 0;
    //foreach (QGraphicsItem *item, scene()->items()) 
    for( auto &[k,node] : dsr_to_graph_viewer->getGMap())
	{
        //GraphNode *node = qgraphicsitem_cast<GraphNode *>(item);
        //if (!node)
        //    continue;
        (void)k;

        QPointF vec = mapToItem(node, 0, 0);
        qreal dx = vec.x();
        qreal dy = vec.y();
        double l = 2.0 * (dx * dx + dy * dy);
        if (l > 0) 
		{
            xvel += (dx * 150.0) / l;
            yvel += (dy * 150.0) / l;
        }
    }

    // Now subtract all forces pulling items together
    double weight = (edgeList.size() + 1) * 10;
    foreach (GraphEdge *edge, edgeList) 
	{
        QPointF vec;
       
        if (edge->sourceNode() == this)
            vec = mapToItem(edge->destNode(), 0, 0);
        else
            vec = mapToItem(edge->sourceNode(), 0, 0);
        xvel -= vec.x() / weight;
        yvel -= vec.y() / weight;
    }

    // Subtract force from central pos pulling item to the center of the image
    QPointF to_central_point = mapFromItem(dsr_to_graph_viewer->getCentralPoint(), 0, 0);
    xvel += to_central_point.x() / (weight/2) ;
    yvel += to_central_point.y() / (weight/2) ;

    // sludge
    if (qAbs(xvel) < 0.1 && qAbs(yvel) < 0.1)
        xvel = yvel = 0;

    QRectF sceneRect = scene()->sceneRect();
    newPos = pos() + QPointF(xvel, yvel);
    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + 10), sceneRect.right() - 10));
    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + 10), sceneRect.bottom() - 10));
}

bool GraphNode::advancePosition()
{
    if (newPos == pos())
        return false;

    setPos(newPos);
    return true;
}

QRectF GraphNode::boundingRect() const
{
    qreal adjust = 2;
    return QRectF( -10 - adjust, -10 - adjust, 23 + adjust, 23 + adjust);
}

QPainterPath GraphNode::shape() const
{
    QPainterPath path;
    path.addEllipse(-10, -10, 20, 20);
    return path;
}

void GraphNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    painter->setPen(Qt::NoPen);
    painter->setBrush(Qt::darkGray);
    painter->drawEllipse(-7, -7, 20, 20);

    QRadialGradient gradient(-3, -3, 10);
    if (option->state & QStyle::State_Sunken)
		{
        gradient.setCenter(3, 3);
        gradient.setFocalPoint(3, 3);
        gradient.setColorAt(1, QColor(Qt::lightGray).light(120));
        gradient.setColorAt(0, QColor(Qt::gray).light(120));
    } else
		{
        gradient.setColorAt(0, QColor(node_brush.color()));
        gradient.setColorAt(1, QColor(node_brush.color().dark(200)));
    }
    painter->setBrush(gradient);
    painter->setPen(QPen(Qt::black, 0));
    painter->drawEllipse(-10, -10, 20, 20);
}

QVariant GraphNode::itemChange(GraphicsItemChange change, const QVariant &value)
{
    switch (change) 
	{
        case ItemPositionChange:
        {
            foreach (GraphEdge *edge, edgeList)
                 edge->adjust(this, value.toPointF());
            break;
        }

        default:
            break;
    };
    return QGraphicsItem::itemChange(change, value);
}


void GraphNode::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event)
{
    //if (tag->text() != "") return; // Explota sin esto
//    animation->start();
    qDebug() << __FILE__ <<":"<<__FUNCTION__<< "-> node: " << tag->text() << " Type: " << QString::fromStdString(type) ;
    const auto graph = dsr_to_graph_viewer->getGraph();
    if( event->button()== Qt::RightButton)
    {
        static std::unique_ptr<QWidget> do_stuff;
        if(type=="laser")
            do_stuff = std::make_unique<DoLaserStuff>(graph, id_in_graph);
        else if(type=="rgbd")
            do_stuff = std::make_unique<DoRGBDStuff>(graph, id_in_graph);
        else
            do_stuff = std::make_unique<DoTableStuff>(graph, id_in_graph);
    }
    update();
    QGraphicsEllipseItem::mouseDoubleClickEvent(event);
}

void GraphNode::change_detected()
{
    animation->start();
}

void GraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
     if( event->button()== Qt::LeftButton)
    {
        auto g = dsr_to_graph_viewer->getGraph();
        qDebug() << __FILE__ <<":"<<__FUNCTION__<< " node id in graphnode: " << id_in_graph ;
        std::optional<Node> n = g->get_node(id_in_graph);
        if (n.has_value()) {
//            qDebug()<<"ScenePos X"<<(float) event->scenePos().x()<<" Width "<<(this->rect())<<" this "<<this->pos().x();
//            qDebug()<<"ScenePos Y"<<(float) event->scenePos().y()<<" Height "<<(this->rect())<<" this "<<this->pos().y();
            bool r = g->modify_attrib_local(n.value(), "pos_x", (float) this->pos().x());
            if (!r) r = g->add_attrib_local(n.value(), "pos_x", (float) this->pos().x());
            r = g->modify_attrib_local(n.value(), "pos_y", (float) this->pos().y());
            if (!r) r = g->add_attrib_local(n.value(), "pos_y", (float) this->pos().y());
            g->update_node(n.value());
        }
//        this->dsr_to_graph_viewer->itemMoved();
    }

    QGraphicsItem::mouseReleaseEvent(event);
}


QColor GraphNode::_node_color()
{
    return this->brush().color();
}
void GraphNode::set_node_color(const QColor& c)
{
    node_brush.setColor(c);
    this->setBrush( node_brush );
}

/////////////////////////////////////////////////////////////////////////////////////////7
////
/////////////////////////////////////////////////////////////////////////////////////////
/* void GraphNode::NodeAttrsChangedSLOT(const DSR::IDType &node, const DSR::Attribs &attr)
 {
	 std::cout << "do cool stuff" << std::endl;
 }
*/
// void GraphNode::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
// {
//    // label = new QTableWidget(graph);
//     //label->setText(tag->text().toStdString());
//     //label->show();
//     //label->exec();
//     std::cout << "entering node: " << tag->text().toStdString() << std::endl;
//     update (boundingRect());
// }

// void GraphNode::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
// {
//     // QDialog *label = new QDialog(graph);
//     // label->exec();
//     //label->close();
//     //lable->delete();
//     std::cout << "exiting node: " << tag->text().toStdString() << std::endl;
//     update (boundingRect());
// }