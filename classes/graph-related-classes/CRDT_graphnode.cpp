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

#include "CRDT_graphnode.h"


GraphNode::GraphNode(std::shared_ptr<DSR::DSRtoGraphViewer> dsr_to_graph_viewer_) : dsr_to_graph_viewer(dsr_to_graph_viewer_)
{
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
    setZValue(-1);
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
        gradient.setColorAt(0, QColor(plain_color));
        gradient.setColorAt(1, QColor(dark_color));
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
                 edge->adjust();
            break;
        }

        default:
            break;
    };
    return QGraphicsItem::itemChange(change, value);
}

void GraphNode::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    //if (tag->text() != "") return; // Explota sin esto
    std::cout << __FILE__ <<":"<<__FUNCTION__<< "-> node: " << tag->text().toStdString() << std::endl;
    const auto graph = dsr_to_graph_viewer->getGraph();
    if( event->button()== Qt::RightButton)
    {
        static std::unique_ptr<QWidget> do_stuff;
        if(type=="laser")
            do_stuff = std::make_unique<DoLaserStuff>(graph, id_in_graph);
        else if(tag->text().contains("rgdb"))
            do_stuff = std::make_unique<DoRGBDStuff>(graph, id_in_graph);
        else
            do_stuff = std::make_unique<DoTableStuff>(graph, id_in_graph);
    }
    
    update();
    QGraphicsItem::mousePressEvent(event);
}

void GraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
     if( event->button()== Qt::LeftButton)
    {
        auto g = dsr_to_graph_viewer->getGraph();
        std::cout << __FILE__ <<":"<<__FUNCTION__<< " node id in graphnode: " << id_in_graph << std::endl;
        std::optional<Node> n = g->get_node(id_in_graph);
        if (n.has_value()) {
            bool r = g->modify_attrib(n.value(), "pos_x", (float) event->scenePos().x());
            if (!r) r = g->add_attrib(n.value(), "pos_x", (float) event->scenePos().x());
            r = g->modify_attrib(n.value(), "pos_y", (float) event->scenePos().y());
            if (!r) r = g->add_attrib(n.value(), "pos_y", (float) event->scenePos().y());
            g->update_node(n.value());
        }
    }

    QGraphicsItem::mouseReleaseEvent(event);
}

void GraphNode::keyPressEvent(QKeyEvent *event) 
{
    // if (event->key() == Qt::Key_Escape)
    // {
    //     if(label != nullptr)
    //     {
    //         label->close();
    //         delete label; 
    //         label = nullptr;
    //     }
    // }
    QGraphicsItem::keyPressEvent(event);
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