#include "graphwidget.h"

#include <QCheckBox>


GraphWidget::GraphWidget(QWidget *parent,QMenu *menu):QGraphicsView(parent)
{
	QVBoxLayout *l;
	if (parent)
	{
		setParent(parent);
		l = new QVBoxLayout(parent);
	}
 	else
	{
		l = new QVBoxLayout();
	}


	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	timerId = 0;
	scene = new QGraphicsScene(this);
	scene->setItemIndexMethod(QGraphicsScene::NoIndex);

	setScene(scene);
	setCacheMode(CacheBackground);
	setViewportUpdateMode(BoundingRectViewportUpdate);
	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	setResizeAnchor(AnchorUnderMouse);

	scale(qreal(0.8), qreal(0.8));
	setWindowTitle(tr("QStateMachine Widget"));


	mode_designer = false;
	selectedNode = NULL;
	if( menu != 0)
	{
		node_menu = menu;
		mode_designer = true;
	}

	nodes_map.clear();
	edges_map.clear();

//      qDebug()<<"graphwidget constructor";

	 //Populate the scene
//     for(int x = 0; x < width; x = x + 25) {
//         for(int y = 0; y < heigth; y = y + 25) {
//
//             if(x % 100 == 0 && y % 100 == 0) {
//                 scene->addRect(x, y, 2, 2);
//
//                 QString pointString;
//                 QTextStream stream(&pointString);
//                 stream << "(" << x << "," << y << ")";
//                 QGraphicsTextItem* item = scene->addText(pointString);
//                 item->setPos(x, y);
//             } else {
//                 scene->addRect(x, y, 1, 1);
//             }
//         }
//     }
//
	l->addWidget(this);
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	resize(700,700);
}

GraphWidget::~GraphWidget()
{
}

void GraphWidget::clear()
{
	foreach (QGraphicsItem *item, scene->items()) {
		scene->removeItem(item);
		//delete item;
	}
}

void GraphWidget::itemMoved()
{
	if (!timerId)
		timerId = startTimer(1000 / 25);
}

void GraphWidget::keyPressEvent(QKeyEvent *event)
{
     switch (event->key()) {
     case Qt::Key_Up:
//          centerNode->moveBy(0, -20);
         break;
     case Qt::Key_Down:
//          centerNode->moveBy(0, 20);
         break;
     case Qt::Key_Left:
//          centerNode->moveBy(-20, 0);
         break;
     case Qt::Key_Right:
//          centerNode->moveBy(20, 0);
         break;
     case Qt::Key_Plus:
    //     scaleView(qreal(1.2));
         break;
     case Qt::Key_Minus:
//         scaleView(1 / qreal(1.2));
         break;
     case Qt::Key_Space:
     case Qt::Key_Enter:
         foreach (QGraphicsItem *item, scene->items()) {
             if (qgraphicsitem_cast<Node *>(item))
                 item->setPos(qrand() % width(), qrand() % height());
         }
         break;
     default:
         QGraphicsView::keyPressEvent(event);
     }
}

void GraphWidget::timerEvent(QTimerEvent *event)
{
	Q_UNUSED(event);

	QList<Node *> nodes;
	foreach (QGraphicsItem *item, scene->items()) {
		if (Node *node = qgraphicsitem_cast<Node *>(item))
			nodes << node;
	}

	static bool ddd = false;
	static QCheckBox *c;
	if (not ddd)
	{
		c = new QCheckBox();
		c->show();
		ddd = true;
	}
	
	if (not c->isChecked())
	foreach (Node *node, nodes)
		node->calculateForces();
		

	bool itemsMoved = false;
	foreach (Node *node, nodes) {
		if (node->advance())
			itemsMoved = true;
	}

	if (!itemsMoved) {
		killTimer(timerId);
		timerId = 0;
	}
}
void GraphWidget::SetCenter(const QPointF& centerPoint) {
	//Get the rectangle of the visible area in scene coords
	QRectF visibleArea = mapToScene(rect()).boundingRect();
	//Get the scene area
	QRectF sceneBounds = sceneRect();

	double boundX = visibleArea.width() / 2.0;
	double boundY = visibleArea.height() / 2.0;
	double boundWidth = sceneBounds.width() - 2.0 * boundX;
	double boundHeight = sceneBounds.height() - 2.0 * boundY;

	//The max boundary that the centerPoint can be to
	QRectF bounds(boundX, boundY, boundWidth, boundHeight);

	currentCenter = centerPoint;

	if(!bounds.contains(centerPoint))
	{//We need to clamp or use the center of the screen
		if(visibleArea.contains(sceneBounds))
			currentCenter = sceneBounds.center();
		else
		{
			//We need to clamp the center. The centerPoint is too large
			if(currentCenter.x() > bounds.x() + bounds.width())
				currentCenter.setX(bounds.x() + bounds.width());
			else if(currentCenter.x() < bounds.x())
				currentCenter.setX(bounds.x());

			if(currentCenter.y() > bounds.y() + bounds.height())
				currentCenter.setY(bounds.y() + bounds.height());
			else if(currentCenter.y() < bounds.y())
				currentCenter.setY(bounds.y());
		}
	}
	centerOn(currentCenter);
}




void GraphWidget::wheelEvent(QWheelEvent *event)
{
	if(event->delta() > 0)
		scale(1/0.95, 1/0.95);
	else
		scale(0.95,0.95);
	currentCenter = mapToScene(rect()).boundingRect().center();
}

void GraphWidget::mousePressEvent(QMouseEvent *event)
{
	bool existItem = false;
	mousePosition = event->pos();

	foreach(QGraphicsItem *item,items(event->pos())){
		if (qgraphicsitem_cast<Node *>(item) != NULL)
			  existItem = true;
	}
	if(existItem)
	{
		event->ignore();
		QGraphicsView::mousePressEvent(event);
	}
	else
	{
		selected_Node(-1);
		if(event->button() == Qt::RightButton and mode_designer)
		{
			scene_menu->exec(event->pos());
		}
		else if(event->button() == Qt::LeftButton)
		{
			LastPanPoint = event->pos();
			setCursor(Qt::ClosedHandCursor);
// 			qDebug()<<"pess event no item";
		}
	}

}
void GraphWidget::mouseReleaseEvent(QMouseEvent* event) {
	if(!LastPanPoint.isNull())
	{
		setCursor(Qt::ArrowCursor);
		LastPanPoint = QPoint();
	}
	else
	{
		event->ignore();
		QGraphicsView::mouseReleaseEvent(event);
	}
}

void GraphWidget::mouseMoveEvent(QMouseEvent* event) {
	if(!LastPanPoint.isNull()) {
// 		QRectF visibleArea = mapToScene(rect()).boundingRect();
		QPointF delta = mapToScene(LastPanPoint) - mapToScene(event->pos());
		LastPanPoint = event->pos();
		SetCenter(currentCenter + delta);
	}
	else
	{
		event->ignore();
		QGraphicsView::mouseMoveEvent(event);
	}
}

void GraphWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
/*
	Q_UNUSED(rect);

	// Shadow
	QRectF sceneRect = this->sceneRect();
	QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
	QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
	if (rightShadow.intersects(rect) || rightShadow.contains(rect))
		painter->fillRect(rightShadow, Qt::darkGray);
	if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
		painter->fillRect(bottomShadow, Qt::darkGray);

	// Fill
	QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
	gradient.setColorAt(0, Qt::white);
	gradient.setColorAt(1, Qt::lightGray);
	painter->fillRect(rect.intersect(sceneRect), gradient);
	painter->setBrush(Qt::NoBrush);
	painter->drawRect(sceneRect);
*/
}

void GraphWidget::resizeEvent ( QResizeEvent * event )
{
	scene->setSceneRect(0,0,event->size().width(),event->size().height());
	itemMoved();
}

void GraphWidget::setActiveNodes(QList<int> nodeList)
{
	foreach(int key,nodes_map.keys())
	{
		if(nodeList.contains(key))
			nodes_map[key]->setActive(true);
		else
			nodes_map[key]->setActive(false);
	}
}


Node* GraphWidget::addNode(QString id,int key,NodeType type)
{
	Node *node=new Node(this,id,key,type,node_menu);
	nodes_map[key] = node;
	scene->addItem(node);
	connect(node,SIGNAL(selectedNode(int)),this,SLOT(selected_Node(int)));
	connect(node,SIGNAL(removeNode(int)),this,SLOT(remove_Node_key(int)));
	return node;
}
Node* GraphWidget::addNode(QString id,int key,int parentKey,NodeType type)
{
	Node *node = NULL;
	if(nodes_map.contains(parentKey))
	{
//		qDebug()<<"widget:: add node "<<id<<key<<"parent"<<parentKey;
		node=new Node(this,id,key,type,node_menu);
		nodes_map[key] = node;
		node->setParentItem(nodes_map[parentKey]);
		nodes_map[parentKey]->expand();
//		qDebug()<<"add node"<<id<<parentKey;
		connect(node,SIGNAL(selectedNode(int)),this,SLOT(selected_Node(int)));
		connect(node,SIGNAL(removeNode(int)),this,SLOT(remove_Node_key(int)));
		nodes_map[parentKey]->hasChild = true;
	}
	else
		qDebug()<<"add node: no parent";
	return node;
}
void GraphWidget::removeNode(Node *n)
{
	n->autoRemove();
	Node* parent = n->getParent();
	if(parent != NULL)
	{
		parent->updateSize();
	}
}

bool GraphWidget::containsNode(int key)
{
	return nodes_map.contains(key);
}

Edge* GraphWidget::addEdge(int source,int target,EdgeType t)
{
	QString key = QString::number(source)+"_"+QString::number(target);
	Edge *edge = NULL;
	if(!edges_map.contains(key))
	{
		if(nodes_map.contains(source) and nodes_map.contains(target))
		{
			Node *parent = dynamic_cast<Node *>(nodes_map[source]->parentItem());
			edge = new Edge(key,nodes_map[source],nodes_map[target],t);
 			if (nodes_map[source]->parentItem() == nodes_map[target]->parentItem() and parent != NULL )
 				edge->setParentItem(parent);
 			else
				scene->addItem(edge);

			edges_map[key] = edge;
			connect(edge,SIGNAL(removeEdge(QString)),this,SLOT(remove_Edge_key(QString)));
		}
	}
	return edge;
}
bool GraphWidget::containsEdge(QString key)
{
	return edges_map.contains(key);
}
Node * GraphWidget::getNode(int key)
{
	if(nodes_map.contains(key))
		return nodes_map[key];
	else
		return NULL;
}
QList<Node *> GraphWidget::getNodes()
{
	return nodes_map.values();

}
void GraphWidget::selected_Edge(QString key)
{
	if(edges_map.contains(activeEdge))
		edges_map[activeEdge]->setActive(false);

	if(edges_map.contains(key))
	{
		activeEdge = key;
		edges_map[activeEdge]->setActive(true);
	}
}



void GraphWidget::selected_Node(int key)
{
	if(selectedNode != NULL and key != selectedNode->getKey())
		selectedNode->setActive(false);
	if(key != -1)
		selectedNode = nodes_map[key];
	else
		selectedNode = NULL;
	emit activeNode(key);
}

Node *GraphWidget::getActiveNode()
{
	return selectedNode;
}
QPointF GraphWidget::getLastMousePosition()
{
	return mousePosition;
}
void GraphWidget::setSceneMenu(QMenu *menu)
{
	scene_menu = menu;
}
void GraphWidget::remove_Node_key(int key)
{
	nodes_map.remove(key);
}
void GraphWidget::remove_Edge_key(QString key)
{
	edges_map.remove(key);
}
void GraphWidget::hideAll()
{
	foreach(QGraphicsItem *item,scene->items()){
		item->setVisible(false);
	}
}
void GraphWidget::showAll()
{
	foreach(QGraphicsItem *item,scene->items()){
		item->setVisible(true);
	}
}
