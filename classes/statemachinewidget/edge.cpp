#include "edge.h"

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;

Edge::Edge(QString _id,Node *sourceNode, Node *destNode,EdgeType t)
{
	setZValue(1);
	setAcceptedMouseButtons(0);
	source = sourceNode;
	dest = destNode;
	source->addEdge(this);
	if(source != dest)
		dest->addEdge(this);
	adjust();
	id = _id;
	arrowSize = 10;
	edgetype = t;
}

Edge::~Edge()
{
}

Node *Edge::sourceNode() const
{
	return source;
}

void Edge::setSourceNode(Node *node)
{
	source = node;
	adjust();
}

Node *Edge::destNode() const
{
	return dest;
}

void Edge::setDestNode(Node *node)
{
	dest = node;
	adjust();
}
void Edge::setActive(bool active)
{
	this->active = active;
}


void Edge::adjust()
{
	if (!source || !dest)
		return;
	if (source != dest)
	{
		QLineF line(mapFromItem(source, source->getSize()/2, source->getSize()/2), mapFromItem(dest, dest->getSize()/2, dest->getSize()/2));
		qreal length = line.length();
		prepareGeometryChange();
		if (length > qreal(20.))
		{
			QPointF edgeOffsetp1((line.dx() * source->getSize()/2) / length, (line.dy() * source->getSize()/2) / length);
			QPointF edgeOffsetp2((line.dx() * dest->getSize()/2) / length, (line.dy() * dest->getSize()/2) / length);
			sourcePoint = line.p1() + edgeOffsetp1;
			destPoint = line.p2() - edgeOffsetp2;
		}
		else
			sourcePoint = destPoint = line.p1();
	}
	else
	{
		prepareGeometryChange();
		float radius = (source->getSize() -2)/2.;
		sourcePoint.setX(radius + cos(M_PI/4.)*radius);
		sourcePoint.setY(radius - cos(M_PI/4.)*radius);
		destPoint = sourcePoint;
		destPoint.setY(radius + cos(M_PI/4.)*radius);
		sourcePoint = mapFromItem(source,sourcePoint);
		destPoint = mapFromItem(source,destPoint);
	}
}

QRectF Edge::boundingRect() const
{
	if (!source || !dest)
		return QRectF();
	
	qreal penWidth = 1;
	qreal extra = (penWidth + arrowSize) / 2.0;
	if(source != dest)
		return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),destPoint.y() - sourcePoint.y())).normalized().adjusted(-extra, -extra, extra, extra);
	else
		return QRectF(sourcePoint,destPoint + QPointF((source->getSize()),extra));
}

void Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	painter->setPen(QPen(Qt::black, 0));
// 	painter->drawRect(boundingRect());
	
	float angle = 0.f;
	bool drawAxis = false;
	if (!source || !dest)
		return;

	QPen pen = QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
	if (active)
		pen.setColor(Qt::yellow);
	else
	{
		if( !source->isVisible() or !dest->isVisible())
		{
			pen = QPen(Qt::blue, 1, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin);
		}	
	}
	if(source != dest )
	{
		QLineF line(sourcePoint, destPoint);
		if (qFuzzyCompare(line.length(), qreal(0.)))
			return;

		// Draw the line itself
		if (edgetype == Error or edgetype == 1)
			pen = QPen(Qt::red, 1, Qt::DashDotDotLine, Qt::RoundCap, Qt::RoundJoin);

		painter->setPen(pen);
		painter->drawLine(line);
		angle = ::acos(line.dx() / line.length());
		if (line.dy() >= 0)
			angle = TwoPi - angle;
		drawAxis = true;
	}
	else if(source->isVisible())
	{
		painter->setPen(pen);
		QPainterPath line(sourcePoint);
		line.cubicTo(sourcePoint+QPointF(source->getSize(),0),destPoint+QPointF(source->getSize(),0),destPoint);
		painter->drawPath(line);	
		angle = -M_PI ;
		drawAxis = true;
	}
	//Arrow
	if(drawAxis)
	{
		if(dest->getSize() < 50 )
			arrowSize = 7;
		else 
			arrowSize = 10;
		QPointF destArrowP1 = destPoint + QPointF(sin(angle - M_PI / 3) * arrowSize,cos(angle - M_PI / 3) * arrowSize);
		QPointF destArrowP2 = destPoint + QPointF(sin(angle - M_PI + M_PI / 3) * arrowSize,cos(angle - M_PI + M_PI / 3) * arrowSize);
		painter->setBrush(Qt::black);
		painter->drawPolygon(QPolygonF() << destPoint << destArrowP1 << destArrowP2);
	}
	
}
void Edge::autoRemove()
{
	qDebug()<<"remove edge"<<id<<source->getId()<<dest->getId();
	source->removeEdge(this);
	dest->removeEdge(this);
	scene()->removeItem(((QGraphicsItem*)this));
	emit removeEdge(id);
}