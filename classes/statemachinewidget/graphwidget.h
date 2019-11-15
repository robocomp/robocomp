
#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QGraphicsView>
#include <QDebug>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <QTimer>
#include <QVBoxLayout>
#include <math.h>
#include "edge.h"
#include "node.h"



class Node;

class GraphWidget : public QGraphicsView
{
Q_OBJECT

public:
	GraphWidget(QWidget *parent=0,QMenu *menu = 0);
	~GraphWidget();
	
	void itemMoved();

//	void checkNewItems();
	void clear();


	void setSceneMenu(QMenu *menu);
	//Nodes
	Node* addNode(QString id,int key,NodeType = Common);
	Node* addNode(QString id,int key,int parentKey,NodeType = Common);
	bool containsNode(int key);
	Node * getNode(int key);
	QList<Node *> getNodes();
	Node * getActiveNode();
	void setActiveNodes(QList<int> nodes);	
	void setSelectableNodes(bool s);
	void removeNode(Node *n);
	//Edges
	Edge* addEdge(int source,int target,EdgeType t);
	bool containsEdge(QString key);
	
	//mouse events
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
 	void wheelEvent(QWheelEvent *event);
	
	QPointF getLastMousePosition();
	
	void hideAll();
	void showAll();
	
	
protected:
    void SetCenter(const QPointF& centerPoint);
	void keyPressEvent(QKeyEvent *event);
	void timerEvent(QTimerEvent *event);
	
	void drawBackground(QPainter *painter, const QRectF &rect);

	void resizeEvent ( QResizeEvent * event );
private:
	int timerId;
	QGraphicsScene *scene;
	QMap<int,Node *> nodes_map;
	QMap<QString,Edge*> edges_map;
	QPointF currentCenter;
	QPoint LastPanPoint;
	QMenu *node_menu;
	QMenu *scene_menu;
	QPointF mousePosition;
	bool mode_designer;
	Node *selectedNode;
	QString activeEdge;
	int lastKey;
	public slots:
		void selected_Node(int key);
		void selected_Edge(QString key);
		void remove_Node_key(int key);
		void remove_Edge_key(QString key);
//		void renewNode(QString key,QPointF);
	signals:
		void activeNode(int);
//		void newNode(QString,QPointF);
};

#endif