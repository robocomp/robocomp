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
#include "innermodeltest.h"

InnerModelTest::InnerModelTest( QObject *parent) : QObject( parent)
{
	/// Create model
	
/*	innerModel->treePrint();

	innerModel->getTransform("leftPan")->print("leftPan");
	innerModel->getTransform("leftMotor")->print("leftMotor");
	innerModel->getTransform("rightMotor")->print("rightMotor");
	innerModel->getTransform("rightPan")->print("rightPan");*/
}

InnerModelTest::~InnerModelTest()
{
	delete innerModel;
}

void InnerModelTest::testUpdateTransform()
{
	innerModel = new InnerModel("../speedyTurtle.xml");
	QVec res1 = innerModel->transform("pen", QVec::vec3(0, 0, 0), "world");
	res1.print("Base desde el punto de vista del pen");
	
	innerModel->updateTransformValues("pen", 0, 0, 200, 0, 0, 0);
	
	QVec res2 = innerModel->transform("pen", QVec::vec3(0, 0, 0), "world");
	res2.print("ahora");
}

// void InnerModelTest::testRightFromLeft()
// {
// 	innerModel = new InnerModel("not_robex.xml");
// 	QVec res = innerModel->getTransformationMatrix("leftCamera", "rightCamera")*QVec::vec4(0, 0, 0, 1);
// 	QCOMPARE((int)res(0), (int)109);
// 	QCOMPARE((int)res(1), (int)0);
// 	QCOMPARE((int)res(2), (int)109);
// }
// 
// void InnerModelTest::testLeftFromRight()
// {
// 	innerModel = new InnerModel("not_robex.xml");
// 	QVec res = innerModel->getTransformationMatrix("rightCamera", "leftCamera")*QVec::vec4(0, 0, 0, 1);
// 	QCOMPARE((int)res(0), (int)-155);
// 	QCOMPARE((int)res(1), (int)0);
// 	QCOMPARE((int)res(2), (int)0);
// }
// 
// void InnerModelTest::testRightPointFromBase()
// {
// 	innerModel = new InnerModel("not_robex2.xml");
// 	QVec res = innerModel->getTransformationMatrix("base", "rightCamera")*QVec::vec4(0, 0, 605.283404696, 1);
// 	res.print("RESS");
// 	QCOMPARE((int)rint(res(0)), (int)78);
// 	QCOMPARE((int)rint(res(1)), (int)0);
// 	QCOMPARE((int)rint(res(2)), (int)428+100);
// }

// void InnerModelTest::testLeftPointFromBase()
// {
// 	innerModel = new InnerModel("not_robex2.xml");
// 	QVec res = innerModel->getTransformationMatrix("base", "leftCamera")*QVec::vec4(0, 0, 605.283404696, 1);
// 	res.print("RESS");
// 	QCOMPARE((int)res(0), (int)-155);
// 	QCOMPARE((int)res(1), (int)0);
// 	QCOMPARE((int)res(2), (int)0);
// 	delete innerModel;
// }

/*
	printf("********************************************* TEST innermodel:\n");
// 	(innerModel->getRotationMatrixTo("leftCamera", "rightCamera")*QVec::vec3(0, 0, 1)).print("(0,0,1)@RightCam  R@  LeftCam");
// 	(innerModel->getRotationMatrixTo("rightCamera", "leftCamera")*QVec::vec3(0, 0, 1)).print("(0,0,1)@LeftCam  R@  RightCam");
void InnerModelTest::testRobotToWorld()
{
	printf("********************************************* TEST mio:\n");
	QCOMPARE(innerModel->robotToWorld(QVec::vec3(1, 0, 0)) , QVec::vec3(0, 0, 0));
}

void InnerModelTest::testWorldToRobot()
{
	printf("********************************************* TEST mio:\n");
	QVec test = innerModel->transform("world", QVec::vec3(0,0,0), "base");
	test.print("test");
}

void InnerModelTest::testTransform_1()
{
	printf("********************************************* TEST mio:\n");
}

void InnerModelTest::testTransform_2()
{
}

*/
