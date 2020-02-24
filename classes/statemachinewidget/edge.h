

#ifndef EDGE_H
#define EDGE_H
#include <QPainter>
#include <QPen>

#include "node.h"

#include <math.h>
#include <QGraphicsItem>
#include <QDebug>

class Node;

enum EdgeType{ Normal, Error };

class Edge : public QObject, public QGraphicsItem
{
Q_OBJECT
public:
     Edge(QString id,Node *sourceNode, Node *destNode,EdgeType t=Normal);
     ~Edge();

     Node *sourceNode() const;
     void setSourceNode(Node *node);

     Node *destNode() const;
     void setDestNode(Node *node);

     void adjust();

	enum { Type = UserType + 2 };
	int type() const { return Type; }
	QString id;
	QString getId(){ return id;}
	void autoRemove();
	void setActive(bool active);


	EdgeType edgetype;


protected:
     QRectF boundingRect() const;
     void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
     Node *source, *dest;

     QPointF sourcePoint;
     QPointF destPoint;
     int arrowSize;
	 bool active;

	signals:
		void removeEdge(QString);

};

#endif