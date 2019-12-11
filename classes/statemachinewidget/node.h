

#ifndef NODE_H
#define NODE_H

#include <QGraphicsItem>
#include <QMenu>
#include <QtCore>
#include <QList>
#include <QDebug>
#include <math.h>
#include <QTime>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>

#include "edge.h"
#include "node.h"

#define ACTIVE_TIME 2000

class Edge;
class GraphWidget;
class QGraphicsSceneMouseEvent;

enum NodeType{ Common,Initial, Final };

class Node : public QObject,public QGraphicsItem
{
Q_OBJECT
public:
	void addChildNode(Node *n) { childNodes.push_back(n); }
	QPointF getNodePos() {return newPos;}
	QList<Node *> childNodes;
	QPointF vel;
	Node(GraphWidget *graphWidget,QString id,int key,NodeType = Common,QMenu *menu=0);
	~Node();
	void addEdge(Edge *edge);
	 void removeEdge(Edge *edge);
	QList<Edge *> edges() const;
	QPointF newPos;

	enum { Type = UserType + 1 };
	int type() const { return Type; }


			/******/
			int idx;
			int area;
			bool fill;
			bool expanded;
			bool active;
			bool hasChild;
			QString id;
			int key;
			QGraphicsTextItem *title;
			NodeType nodetype;
			/*****/

	void calculateForces();
	bool advance();

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void expand();
	void contract();
	void updateSize(bool first=false);

	void setId(QString id);
	QString getId();
	int getKey();
	void setParentItem(Node *parent);
	void setType(NodeType _type);
	NodeType getType();
	Node* getParent();
	void autoRemove();
	void hide();
	void show();
	int getSize() {return size;}
 protected:
			int size;
	void computeMaxDistRad(float &maxDist, float &maxRad);
	void computeChildArea(float &areaC);
     QVariant itemChange(GraphicsItemChange change, const QVariant &value);

     void mousePressEvent(QGraphicsSceneMouseEvent *event);
     void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
//      void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

	public slots:
		void setActive(bool active=true);
		void setInactive() { setActive(false); }
	signals:
		void selectedNode(int);
		void removeNode(int);
		void removeEdge(QString);
 private:
     QList<Edge *> edgeList;
     GraphWidget *graph;
	 QTime time;
	 QMenu *action_menu;
	 bool mode_designer;
	 Node* parent;
};
#endif
