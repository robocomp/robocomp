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

#include <dsr/gui/viewers/graph_viewer/graph_node.h>
//#include <dsr/gui/viewers/graph_viewer/node_colors.h>
#include <dsr/gui/viewers/graph_viewer/graph_colors.h>
#include <dsr/gui/viewers/graph_viewer/graph_node_laser_widget.h>
#include <dsr/gui/viewers/graph_viewer/graph_node_widget.h>
#include <dsr/gui/viewers/graph_viewer/graph_node_rgbd_widget.h>


GraphNode::GraphNode(const std::shared_ptr<DSR::GraphViewer>&
        graph_viewer_):QGraphicsEllipseItem(0,0,DEFAULT_DIAMETER,DEFAULT_DIAMETER), graph_viewer(graph_viewer_)
{
    auto flags = ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges | ItemUsesExtendedStyleOption | ItemIsFocusable;
    setFlags(flags);
    setCacheMode(DeviceCoordinateCache);
    setAcceptHoverEvents(true);
    setZValue(-1);
    node_brush.setStyle(Qt::SolidPattern);

    animation = new QPropertyAnimation(this, "node_color");
	animation->setDuration(animation_time);
	animation->setStartValue(plain_color);
	animation->setEndValue(dark_color);
	animation->setLoopCount(ANIMATION_REPEAT);
    QObject::connect(graph_viewer_->getGraph().get(), &DSR::DSRGraph::update_node_attr_signal, this, &GraphNode::update_node_attr_slot);
}

void GraphNode::setTag(const std::string &tag_)
{
    QString c = QString::fromStdString(tag_);
	tag = new QGraphicsSimpleTextItem(c, this);
	tag->setX(DEFAULT_DIAMETER);
	tag->setY(-10);
}

void GraphNode::setType(const std::string &type_)
{
    type = type_;
    if(type == "laser" or type == "rgbd")
    {
        contextMenu = new QMenu();
        QAction *table_action = new QAction("View table");
        contextMenu->addAction(table_action);
        connect(table_action, &QAction::triggered, this, [this](){ this->show_node_widget("table");});
        QAction *stuff_action = new QAction("View data");
        contextMenu->addAction(stuff_action);
        connect(stuff_action, &QAction::triggered, this, [this, type_](){ this->show_node_widget(type_);});
    }
    auto color = GraphColors<DSR::Node>()[type];
    set_color(color);
}


void GraphNode::addEdge(GraphEdge *edge)
{
    qDebug()<<"====================================";
    int same_count = 0;
    int bend_factor = 0;
    // look for edges with the same dest
    qDebug()<<__FUNCTION__ <<"Checking edges for node: "<<this->id_in_graph;
    for (auto old_edge: edgeList)
    {
        if(old_edge == edge)
            throw std::runtime_error("Trying to add an already existing edge " + std::to_string(edge->sourceNode()->id_in_graph)+"--"+std::to_string(edge->destNode()->id_in_graph));
//        qDebug()<<__FUNCTION__ <<"\tExisting EDGE: "<<edge->sourceNode()->id_in_graph<<edge->destNode()->id_in_graph<<"OTHER: "<<old_edge->sourceNode()->id_in_graph<<old_edge->destNode()->id_in_graph;
        qDebug()<<"\t"<<__FUNCTION__ <<"Existing EDGE: "<<old_edge->sourceNode()->id_in_graph<<"--"<<old_edge->destNode()->id_in_graph;
        qDebug()<<"\t"<<__FUNCTION__ <<"     New EDGE: "<<edge->sourceNode()->id_in_graph<<"--"<<edge->destNode()->id_in_graph;
        if((edge->sourceNode()->id_in_graph==old_edge->sourceNode()->id_in_graph or edge->sourceNode()->id_in_graph==old_edge->destNode()->id_in_graph)
        and (edge->destNode()->id_in_graph==old_edge->sourceNode()->id_in_graph or edge->destNode()->id_in_graph==old_edge->destNode()->id_in_graph))
        {
            same_count++;
            qDebug()<<"\t\t"<<__FUNCTION__ <<"SAME EDGE"<<same_count;
        }
    }
    //            https://www.wolframalpha.com/input/?i=0%2C+1%2C+-1%2C+2%2C+-2%2C+3%2C+-3
    bend_factor = (pow(-1,same_count)*(-1 + pow(-1,same_count) - 2*same_count))/4;
    qDebug()<<__FUNCTION__ <<__LINE__<<"ID: "<<id_in_graph<<"SAME: "<<same_count<<"FACTOR: "<<bend_factor;
    edge->set_bend_factor(bend_factor);
    edgeList << edge;

    edge->adjust();
    qDebug()<<"====================================";

}

void GraphNode::deleteEdge(GraphEdge *edge)
{
    edgeList.removeOne(edge);
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
    for( auto &[k,node] : graph_viewer->getGMap())
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
            xvel += (dx * force_velocity_factor) / l;
            yvel += (dy * force_velocity_factor) / l;
        }
    }

    // Now subtract all forces pulling items together
    double weight = (edgeList.size() + 1) * EDGE_PULL_FACTOR;
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
    QPointF to_central_point = mapFromItem(graph_viewer->getCentralPoint(), 0, 0);
    xvel += to_central_point.x() / (weight/2) ;
    yvel += to_central_point.y() / (weight/2) ;

    // sludge
    if (qAbs(xvel) < 0.1 && qAbs(yvel) < 0.1)
        xvel = yvel = 0;

    QRectF sceneRect = scene()->sceneRect();
    newPos = pos() + QPointF(xvel, yvel);
    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + SCENE_MARGIN), sceneRect.right() - SCENE_MARGIN));
    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + SCENE_MARGIN), sceneRect.bottom() - SCENE_MARGIN));
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
    return QRectF(
            -DEFAULT_RADIUS - adjust,
            -DEFAULT_RADIUS - adjust,
            DEFAULT_DIAMETER + 3 + adjust,
            DEFAULT_DIAMETER + 3 + adjust
            );
}

QPainterPath GraphNode::shape() const
{
    QPainterPath path;
    path.addEllipse(-DEFAULT_RADIUS, -DEFAULT_RADIUS, DEFAULT_DIAMETER, DEFAULT_DIAMETER);
    return path;
}

void GraphNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    painter->setPen(Qt::NoPen);
    painter->setBrush(SUNKEN_COLOR);
//    painter->drawEllipse(-7, -7, node_width, node_width);

    QRadialGradient gradient(-3, -3, 10);
    if (option->state & QStyle::State_Sunken)
    {
        gradient.setColorAt(0, QColor(Qt::darkGray).light(LUMINOSITY_FACTOR));
        gradient.setColorAt(1, QColor(Qt::darkGray));
    } else
		{
        gradient.setColorAt(0, node_brush.color());
        gradient.setColorAt(1, QColor(node_brush.color().dark(LUMINOSITY_FACTOR)));
    }
    painter->setBrush(gradient);
    if(isSelected())
        painter->setPen(QPen(Qt::green, 0, Qt::DashLine));
    else
        painter->setPen(QPen(Qt::black, 0));
    painter->drawEllipse(-DEFAULT_RADIUS, -DEFAULT_RADIUS, DEFAULT_DIAMETER, DEFAULT_DIAMETER);
}

QVariant GraphNode::itemChange(GraphicsItemChange change, const QVariant &value)
{
    switch (change) 
	{

        case ItemPositionHasChanged:
        {
            foreach (GraphEdge *edge, edgeList)
                 edge->adjust(this, value.toPointF());
            break;
        }

        default:
            break;
    }
    return QGraphicsItem::itemChange(change, value);
}


void GraphNode::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event)
{
    //if (tag->text() != "") return; // Explota sin esto
//    animation->start();
    if( event->button()== Qt::RightButton)
    {
        if (contextMenu != nullptr)
            contextMenu->exec(event->screenPos());
        else
            show_node_widget("table");
    }
//    update();
    QGraphicsEllipseItem::mouseDoubleClickEvent(event);
}


void GraphNode::show_node_widget(const std::string &show_type)
{
    //static std::unique_ptr<QWidget> do_stuff;
    const auto graph = graph_viewer->getGraph();
    if(show_type=="laser")
        node_widget = std::make_unique<GraphNodeLaserWidget>(graph, id_in_graph);
    else if(show_type=="rgbd")
        node_widget = std::make_unique<GraphNodeRGBDWidget>(graph, id_in_graph);
    else
        node_widget = std::make_unique<GraphNodeWidget>(graph, id_in_graph);

}

void GraphNode::change_detected()
{
    animation->start();
}

void GraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
     if( event->button()== Qt::LeftButton)
    {
        auto g = graph_viewer->getGraph();
        qDebug() << __FILE__ <<":"<<__FUNCTION__<< " node id in graphnode: " << id_in_graph ;
        std::optional<Node> n = g->get_node(id_in_graph);
        if (n.has_value()) {
//            qDebug()<<"ScenePos X"<<(float) event->scenePos().x()<<" Width "<<(this->rect())<<" this "<<this->pos().x();
//            qDebug()<<"ScenePos Y"<<(float) event->scenePos().y()<<" Height "<<(this->rect())<<" this "<<this->pos().y();
            g->add_or_modify_attrib_local<pos_x_att>(n.value(), (float) this->pos().x());
            g->add_or_modify_attrib_local<pos_y_att>(n.value(),  (float) this->pos().y());
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

void GraphNode::set_color(const std::string &plain)
{
    QString c = QString::fromStdString(plain);
    plain_color = c;
    dark_color = "dark" + c;
    set_node_color(QColor(c));
    animation->setStartValue(QColor("green").lighter());
    animation->setEndValue(c);
}

/////////////////////////////////////////////////////////////////////////////////////////7
////
/////////////////////////////////////////////////////////////////////////////////////////
void GraphNode::update_node_attr_slot(std::uint64_t node_id, const std::vector<std::string> &type)
{
    if (node_id != this->id_in_graph)
        return;
//    if(std::find(type.begin(), type.end(), "color") != type.end())
//    {
//        std::optional<Node> n = graph_viewer->getGraph()->get_node(node_id);
//        if (n.has_value()) {
////            auto &attrs = n.value().attrs();
////            auto value = attrs.find("color");
////            if (value != attrs.end()) {
////                this->setColor(value->second.str());
////            }
//        }
//    }
}
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
