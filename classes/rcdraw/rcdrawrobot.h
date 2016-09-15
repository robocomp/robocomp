/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RCDRAWROBOT_H
#define RCDRAWROBOT_H

#include "rcdraw.h"
#include <innermodel/innermodel.h>

/**
	@author authorname <authormail>

  The constructor will automatically call show() method.

*/
class RCDrawRobot : public RCDraw
{
Q_OBJECT
public:
	RCDrawRobot(QWidget *parent);
	RCDrawRobot(const QRect & win_, QWidget *parent = 0);
	RCDrawRobot(int _width, int _height, QWidget *parent = 0);
	RCDrawRobot(int _width, int _height, uchar *img, QWidget *parent = 0);
	RCDrawRobot(const QPointF &center, QSize size, QWidget *parent);

	void setImage(QImage *image) { qimg = image; }
	void init();
	void setRobotLimits(QRectF limits);
	void drawRobot(InnerModel *innerModel, const QColor &color = Qt::green) ;
	void drawRobotTrail(InnerModel *innerModel, int QUEUE_SIZE = 100);
	void clearRobotTrail();
// 	void drawRightFieldOfView (InnerModel *innerModel);
// 	void drawLeftFieldOfView (InnerModel *innerModel);
	
private:
	QRectF robotLimits;
	QQueue<QPointF> trajec;
};
#endif
