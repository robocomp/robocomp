#include "rcdrawrobot.h"

RCDrawRobot::RCDrawRobot(QWidget *parent) : RCDraw(parent)
{
	init();
}

RCDrawRobot::RCDrawRobot(const QRect & win_, QWidget *parent) : RCDraw(win_, parent)
{
	init();
}

RCDrawRobot::RCDrawRobot(const QPointF &center, QSize size, QWidget *parent) : RCDraw(QRect(center.x()-(size.width()/2.), center.y()-(size.height()/2.), size.width(), size.height()), parent)
{
	init();
}

RCDrawRobot::RCDrawRobot(int _width, int _height, QWidget *parent) : RCDraw(_width,_height,parent)
{
	init();
}

RCDrawRobot::RCDrawRobot(int _width, int _height, uchar *img, QWidget *parent) : RCDraw(_width, _height, img, parent)
{
	init();
}


void RCDrawRobot::init()
{
	invertedVerticalAxis = false;
	robotLimits.setX(win.x());
	robotLimits.setY(-win.y());
	robotLimits.setWidth(win.width());
	robotLimits.setHeight(-win.height());
}

void RCDrawRobot::setRobotLimits(QRectF limits)
{
	robotLimits = limits;
}

void RCDrawRobot::drawRobot(InnerModel * innerModel, const QColor &color)
{
 	QVec p1 (3);
	int radio = 320;

	//Body
	QVec geomCenter = innerModel->transform("world", QVec::vec3(0,0,0), "robotGeometricCenter");
	QVec leftWheelP1 = innerModel->transform("world",  QVec::vec3(-radio-50, 0, 75), "base");
	QVec leftWheelP2 = innerModel->transform("world",  QVec::vec3(-radio-50, 0, -75), "base");
	QVec headRobot = innerModel->transform("world",  QVec::vec3(0, 0, radio), "base" );
	QLine leftWheel(QPoint(leftWheelP1.x(), leftWheelP1.z()),QPoint(leftWheelP2.x(),leftWheelP2.z()));
	QVec rightWheelP1 = innerModel->transform("world",  QVec::vec3(radio+50, 0, 75), "base");
	QVec rightWheelP2 = innerModel->transform("world",  QVec::vec3(radio+50, 0, -75), "base");
	QLine rightWheel(QPoint(rightWheelP1.x(), rightWheelP1.z()),QPoint(rightWheelP2.x(),rightWheelP2.z()));

	QMat m = innerModel->getRotationMatrixTo("world", "base");
	drawSquare( QPointF( (int) rint(geomCenter(0)), (int) rint(geomCenter(2)) ), 2.*radio, 2.*radio, color, true, -1, atan2f(m(2,0),m(0,0)));
	drawSquare( QPointF( headRobot.x(), headRobot.z() ), radio/3, radio/3, Qt::magenta, true );
// 	drawEllipse( QPointF( innerModel->getBaseX(), innerModel->getBaseZ() ), radio/6, radio/6, Qt::magenta, true );

	//Wheels
	drawLine( leftWheel, Qt::black, 60);
	drawLine( rightWheel, Qt::black, 60);

// 	drawLeftFieldOfView(innerModel);
// 	drawRightFieldOfView(innerModel);

	if((geomCenter(0)-visibleCenter(0))<win.x())
		visibleCenter(0)=geomCenter(0);
	if((geomCenter(0)-visibleCenter(0))>win.x()+win.width())
		visibleCenter(0)=geomCenter(0);
	if((geomCenter(2)-visibleCenter(1))<-win.y())
		visibleCenter(1)=geomCenter(2);
	if((geomCenter(2)-visibleCenter(1))>-win.y()-win.height())
		visibleCenter(1)=geomCenter(2);
}

void RCDrawRobot::drawRobotTrail(InnerModel * innerModel, int QUEUE_SIZE)
{
	QVec p1 (3);
	int radio = 320;

	//Body
	QVec geomCenter = innerModel->transform("world", QVec::vec3(0, 0, -radio), "base");
	QVec leftWheelP1 = innerModel->transform("world",  QVec::vec3(-radio, 0, 75), "base");
	QVec leftWheelP2 = innerModel->transform("world",  QVec::vec3(-radio, 0, -75), "base");
	QLine leftWheel(QPoint(leftWheelP1.x(), leftWheelP1.z()),QPoint(leftWheelP2.x(),leftWheelP2.z()));
	QVec rightWheelP1 = innerModel->transform("world",  QVec::vec3(radio, 0, 75), "base");
	QVec rightWheelP2 = innerModel->transform("world",  QVec::vec3(radio, 0, -75), "base");
	QLine rightWheel(QPoint(rightWheelP1.x(), rightWheelP1.z()),QPoint(rightWheelP2.x(),rightWheelP2.z()));

	//drawEllipse( QPointF( b(0),b(2) ) , radio, radio, Qt::red );
	QVec base  = innerModel->transform6D("world", "base");
	drawSquare( QPointF( (int) rint(geomCenter(0)), (int) rint(geomCenter(2)) ), 2*radio, 2*radio, Qt::green, true, -1, -base(4) );
	drawEllipse( QPointF(base(0), base(2) ), radio/6, radio/6, Qt::red, true );

	//Wheels
	drawLine( leftWheel, Qt::black, 60);
	drawLine( rightWheel, Qt::black, 60);

	if((geomCenter(0)-visibleCenter(0))<win.x())
		visibleCenter(0)=geomCenter(0);
	if((geomCenter(0)-visibleCenter(0))>win.x()+win.width())
		visibleCenter(0)=geomCenter(0);
	if((geomCenter(2)-visibleCenter(1))<-win.y())
		visibleCenter(1)=geomCenter(2);
	if((geomCenter(2)-visibleCenter(1))>-win.y()-win.height())
		visibleCenter(1)=geomCenter(2);
	
	
	trajec.enqueue(QPointF(base(0), base(2)));
	if( trajec.size() > QUEUE_SIZE and trajec.isEmpty()==false) 
		trajec.dequeue();
	foreach(QPointF p, trajec)
	{
		drawSquare(p,10,10,Qt::green,true);	
	}
}

void RCDrawRobot::clearRobotTrail()
{
	trajec.clear();
}

/*
void RCDrawRobot::drawLeftFieldOfView(InnerModel *innerModel)
{
	QVec origen = innerModel->transform("world", QVec::vec3(-80,0,0), "base");
	QVec finalI = innerModel->imageCoordPlusDepthTo("leftCamera", QVec(QPoint(0,0)), 4000.f, "world");
	QVec finalD = innerModel->imageCoordPlusDepthTo("leftCamera", QVec(QPoint(320,0)), 4000.f, "world");

	drawLine( QLineF( QPointF(origen.x(),origen.z()), QPointF(finalI.x(),finalI.z()) ), Qt::green, 15);
	drawLine( QLineF( QPointF(origen.x(),origen.z()), QPointF(finalD.x(),finalD.z()) ), Qt::green, 15);
}

void RCDrawRobot::drawRightFieldOfView(InnerModel *innerModel)
{
	QVec origen = innerModel->transform("world", QVec::vec3(80,0,0), "base");
	QVec finalI = innerModel->imageCoordPlusDepthTo("rightCamera", QVec(QPoint(0,0)), 4000.f, "world");
	QVec finalD = innerModel->imageCoordPlusDepthTo("rightCamera", QVec(QPoint(320,0)), 4000.f, "world");

	drawLine( QLineF( QPointF(origen.x(),origen.z()), QPointF(finalI.x(),finalI.z()) ), Qt::magenta, 15);
	drawLine( QLineF( QPointF(origen.x(),origen.z()), QPointF(finalD.x(),finalD.z()) ), Qt::magenta, 15);
}
*/
