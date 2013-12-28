/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifndef INNERMODELTEST_H
#define INNERMODELTEST_H

#include <QtCore>
#include <QtTest>

#include <qmatriz/QMatAll>

#include "../innermodel.h"

class InnerModelTest : public QObject
{
Q_OBJECT

public:
	InnerModelTest( QObject *parent=0);	
	~InnerModelTest();

	InnerModel *innerModel;
	
private slots:
	void testUpdateTransform();
/*	void testRightFromLeft();
	void testLeftFromRight();
	void testRightPointFromBase();*/
// 	void testLeftPointFromBase();
	
/*	void testRobotToWorld();
	void testWorldToRobot();
	void testTransform_1();
	void testTransform_2();*/
	
};

#endif
