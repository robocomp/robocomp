
#include "graphwidget.h"

Node::Node(GraphWidget *graphWidget,QString _id,int _key,NodeType _type,QMenu *menu) :QObject(), graph(graphWidget)
{
	setFlag(ItemIsMovable);
	setFlag(ItemSendsGeometryChanges);
	setCacheMode(DeviceCoordinateCache);
	setZValue(-1);

	vel.setX(0);
	vel.setY(0);

	this->id = _id;
	key = _key;
	nodetype = _type;
	updateSize(true);
	fill = true;
	expanded = true;
	active = false;
	mode_designer = false;
	hasChild = false;
	parent = NULL;

	if(menu != 0)
	{
		mode_designer = true;
		action_menu = menu;
	}

	title = new QGraphicsTextItem(id,this);
	title->adjustSize();
	int width = graph->width();
	int height = graph->height();
	if(parentItem())
	{
		width = parentItem()->boundingRect().width();
		height = parentItem()->boundingRect().height();
	}
	this->setPos(qrand() % width, qrand() % height);
	time.start();
}
Node::~Node()
{

}
void Node::addEdge(Edge *edge)
{
	edgeList << edge;
	edge->adjust();
}
void Node::removeEdge(Edge *edge)
{
	edgeList.removeOne(edge);
}
QList<Edge *> Node::edges() const
{
	return edgeList;
}

void Node::computeMaxDistRad(float &maxDist, float &maxRad)
{
	maxDist=0;              // Get max distance between childs
	maxRad=0;               // Get max child radius
	QPointF dif;
	foreach (Node *node1, childNodes)
	{
		if (node1->size>maxRad)
			maxRad=node1->size;
		foreach (Node *node2, childNodes)
		{
			if (node1 == node2) continue;
			dif = node1->getNodePos()-node2->getNodePos();
			float trueLength = sqrt(pow(dif.x(), 2) + pow(dif.y(), 2));
			if (trueLength>maxDist) maxDist = trueLength;
		}
	}
}

void Node::computeChildArea(float &areaC)
{
	areaC=0;
	foreach (Node *node1, childNodes)
	{
		areaC += node1->area;
	}
}

void Node::calculateForces(){
	if (!scene() || scene()->mouseGrabberItem() == this) {
        newPos = pos();
        return;
    }

    // Sum up all forces pushing this item away
    qreal xvel = 0;
    qreal yvel = 0;
    const QList<QGraphicsItem *> items = scene()->items();
    for (QGraphicsItem *item : items) {
        Node *node = qgraphicsitem_cast<Node *>(item);
        if (!node)
            continue;

        QPointF vec = mapToItem(node, 0, 0);
        qreal dx = vec.x();
        qreal dy = vec.y();
        double l = 0.5 * (dx * dx + dy * dy);
        if (l > 0) {
            xvel += (dx * 150.0) / l;
            yvel += (dy * 150.0) / l;
        }
    }
    // Now subtract all forces pulling items together
    double weight = (edgeList.size() + 1) * 10;
    for (const Edge *edge : qAsConst(edgeList)) {
        QPointF vec;
        if (edge->sourceNode() == this)
            vec = mapToItem(edge->destNode(), 0, 0);
        else
            vec = mapToItem(edge->sourceNode(), 0, 0);
        xvel -= vec.x() / weight;
        yvel -= vec.y() / weight;
    }
    if (qAbs(xvel) < 0.2 && qAbs(yvel) < 0.2)
        xvel = yvel = 0;

    QRectF sceneRect = scene()->sceneRect();
    newPos = pos() + QPointF(xvel, yvel);
    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + 10), sceneRect.right() - 10));
    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + 10), sceneRect.bottom() - 10));

}

bool Node::advance()
{
	if (newPos == pos())
		return false;
	setPos(newPos);
	return true;
}

QRectF Node::boundingRect() const
{
	qreal adjust = 2;
	return QRectF(0- adjust, 0 - adjust,size+adjust,size+  adjust);
}

QPainterPath Node::shape() const
{
	QPainterPath path;
	path.addEllipse(0, 0, size, size);
	return path;
}
void Node::setActive(bool _active)
{
	if(_active)
		time.restart();
	active = _active;
	update();
}
void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
	//draw bounding rect
	// painter->setPen(QPen(Qt::black, 0));
	// painter->drawRect(boundingRect());

	updateSize();

	painter->setPen(Qt::NoPen);
	painter->setBrush(Qt::darkGray);
	painter->drawEllipse(0, 0, size, size);

	float p =size/2. - 0.7 * size/2.;
	QRadialGradient gradient(p,p, size*3/4);
	gradient.setFocalPoint(10,10);
	if (active) {
		gradient.setColorAt(0, QColor(Qt::yellow));
		gradient.setColorAt(1, QColor(Qt::darkYellow));
	}
	else
	{
		int v = 0;
		if(time.elapsed() <= ACTIVE_TIME)
			v =255. - 255. * ((float)time.elapsed() / ACTIVE_TIME);
		gradient.setColorAt(0, QColor(255,v,0));
		gradient.setColorAt(1, QColor(255,v,0).lighter(50));
	}
	QPen pen = QPen(Qt::black);
	pen.setWidth(1);
	if(nodetype == Final)
	{
		pen.setStyle(Qt::DotLine );
		pen.setColor(Qt::blue);
	}
	else if (nodetype == Initial)
	{
		QVector<qreal> dash;
		dash << 3 << 6 << 1 << 5;
		pen.setDashPattern(dash);
		pen.setColor(Qt::yellow);
	}
	else
		pen.setStyle(Qt::SolidLine );
	painter->setPen(pen);
	if(!expanded)
	{
		painter->setOpacity(1.f);
		if(hasChild)
		{
			QPen p = QPen(Qt::green);
			p.setCapStyle(Qt::RoundCap);
			p.setWidth(2);
			painter->setPen(p);
			painter->drawLine(QPoint(size-4,0),QPoint(size-4,4));
			painter->drawLine(QPoint(size-6,2),QPoint(size-2,2));
			painter->setPen(pen);
		}
	}
	else
		painter->setOpacity(0.4);

	painter->setBrush(gradient);
	painter->drawEllipse(0, 0, size-2, size-2);
	//Title
	title->setPos(-(title->boundingRect().width()-size)/2, -title->boundingRect().height());
}

QVariant Node::itemChange(GraphicsItemChange change, const QVariant &value)
{
	switch (change) {
	case ItemPositionHasChanged:
		foreach (Edge *edge, edgeList)
			edge->adjust();
		graph->itemMoved();
		break;
	default:
		break;
	};

	return QGraphicsItem::itemChange(change, value);
}

void Node::updateSize(bool first)
{
	prepareGeometryChange();

	float maxDist, maxRad;
	float childArea;
	if (size == 0 or first)
		size = 50;
	computeMaxDistRad(maxDist, maxRad);
	computeChildArea(childArea);
//printf("updateSize %s: %f %f %d\n", id.toStdString().c_str(), maxDist, maxRad, size);
	int backSize = size;
	if(expanded)
	{
		if ((maxDist+maxRad > size and size < 500) or childArea/area > 0.5)
		{
			size += 2;
			newPos = newPos + QPointF(1, 1);
		}
		else if (maxDist+maxRad+ 10 < size and size > 50)
		{
			size -= 2;
			newPos = newPos - QPointF(1,1);
		}
		
	}
	area = 3.1459926535*size*size;
	if(parentItem() !=0)
		((Node *)parentItem())->updateSize();
}


void Node::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if(mode_designer)
	{
		active = true;
		emit selectedNode(key);
		update();
		if(event->button() == Qt::RightButton)
			action_menu->exec(event->screenPos());
	}

	QGraphicsItem::mousePressEvent(event);
// 	qDebug()<<"mouse click"<<id<<key;
}

void Node::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	update();
	QGraphicsItem::mouseReleaseEvent(event);
	graph->itemMoved();
}
void Node::setParentItem(Node *parent)
{
	this->parent = parent;
	parent->addChildNode(this);
	QGraphicsItem::setParentItem(parent);
	int width = graph->width();
	int height = graph->height();
	if(parentItem())
	{
		width = parentItem()->boundingRect().width();
		height = parentItem()->boundingRect().height();
	}
	this->setPos(qrand() % width, qrand() % height);
}
QString Node::getId()
{
	return id;
}
int Node::getKey()
{
	return key;
}
void Node::setType(NodeType _type)
{
	nodetype = _type;
}
NodeType Node::getType()
{
	return nodetype;
}

void Node::setId(QString id)
{
	this->id = id;
	title->setPlainText(id);
	title->adjustSize();
}
Node* Node::getParent()
{
	return parent;
}
void Node::autoRemove()
{
	foreach(Edge *e,edgeList)
	{
		e->autoRemove();
	}
	Node *aux;
	foreach(QGraphicsItem *item,childItems())
	{
		aux = qgraphicsitem_cast<Node *>(item);
		if(aux != NULL)
			aux->autoRemove();
	}
	scene()->removeItem(((QGraphicsItem*)this));
	emit removeNode(key);
}

void Node::hide()
{
// 	this->contract();
// 	this->size = 0;
// 	this->setPos(size/2,size/2);
// 	title->setVisible(false);
// 	this->setVisible(false);
// 	foreach(Edge *edge,edgeList)
// 	{
// 		edge->setVisible(false);
// 	}

//	qDebug()<<"hide"<<id;
}

void Node::show()
{
	title->setVisible(true);
	this->setVisible(true);
	updateSize();
	this->setPos(qrand() % size, qrand() % size);
	this->fill = true;
	foreach(Edge *edge,edgeList)
	{
		if(edge->sourceNode()->isVisible() or edge->destNode()->isVisible())
			edge->setVisible(true);
	}
//	qDebug()<<"show"<<id;
}

//not used in this initial version ==> Allow to expand nodes with internal states
void Node::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
// 	qDebug()<<"doble click"<<id<<key;
// 	if(!expanded)
// 		expand();
// 	else
// 		contract();
}

void Node::expand()
{
// 	if(!expanded)
// 	{
// 		bool hasChildren = false;
// 		Node *aux;
// 		foreach(QGraphicsItem *item,childItems()){
// 			if ((aux=qgraphicsitem_cast<Node *>(item)) != NULL)
// 				hasChildren = true;
// 		}
// 		if(hasChildren)
// 		{
// 			expanded = true;
// 			fill = false;
// 			foreach(QGraphicsItem *item,childItems()){
// 				if ((aux=qgraphicsitem_cast<Node *>(item)) != NULL)
// 				{
// 					aux->show();
// 				}
// 			}
// 			updateSize();
// 		}
// 	}
}

// void Node::contract()
// {
// 	if(expanded)
// 	{
// 		expanded = false;
// 		fill = true;
// 		// SIZE XXX
// 		if( parentItem() != 0)
// 			size = 30;
// 		else
// 			size = 50;
// 		updateSize();
// 		foreach(QGraphicsItem *item,childItems()){
// 			Node *node = qgraphicsitem_cast<Node *>(item);
// 			if(node)
// 			{
// 				node->hide();
// 			}
// 		}
// 		title->setVisible(true);
// 	}
// }
