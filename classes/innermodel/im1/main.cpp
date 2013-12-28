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
#include <QApplication>

#include "innermodelreader.h"

int main(int argc, char **argv)
{
	QApplication a(argc, argv);

	/// Create model
	InnerModel *model = new InnerModel("robex.xml");
	model->treePrint();

	/// Set update vectors (initial value)
// 	QVec tUVector = QVec::vec3(0, 120, 0);
// 	QVec tLVector = QVec::vec3(0, -120, 0);




// 	model->setUpdateTranslationPointers("upperMotor", &tUVector(0), &tUVector(1), &tUVector(2));
// 	model->setUpdateTranslationPointers("lowerMotor", &tLVector(0), &tLVector(1), &tLVector(2));



	/// Update tree
//	model->update();
//
	/// Outputs
// 	printf("de la base al mundo\n");
// 	model->transform("base", QVec::vec3(0, 0, 0), "world").print("pose del robot");
// 	QVec r1 = model->robotToWorld( QVec::vec3(-200, 0, 0));
// 	r1.print("r1");
// 	QVec r2 = model->robotToWorld( QVec::vec3(+200, 0, 0));
// 	r2.print("r21");
// 	printf("\n\nde la camara de arriba a la de abajo\n");
// 	model->transform("upperCamera", QVec::vec3(0,0,0), "lowerCamera").print("La posicion de la camara de arriba desde la camara de abajo");



// 	model->project("upperCamera", QVec::vec3(0,0,100), "upperCamera").print("Punto alejao del motor de arriba 100 palante cae en");
// 	model->project("upperMotor", QVec::vec3(0,0,100), "upperCamera").print("Punto alejao de la camara de arriba 100 palante cae en");

// 	model->transform("lowerCamera", QVec::vec3(0,0,0), "upperCamera").print("La posicion de la camara de abajo desde la camara de arriba");
// 	model->project("lowerCamera", QVec::vec3(0,0,100), "lowerCamera").print("Punto alejao del motor de abajo 100 palante cae en");
// 	model->project("lowerMotor", QVec::vec3(0,0,100), "lowerCamera").print("Punto alejao de la camara de abajo 100 palante cae en");

// 	model->horizonLine("floor", "upperCamera").print("upper horizon");
// 	model->horizonLine("floor", "lowerCamera").print("lower horizon");
// 	model->horizonLine("floor", "lowerCamera").print("lower horizon");
	
// 	model->transform("upperCamera", QVec::vec3(0, 0, 0), "world").print("pose de uC");
// 	model->transform("lowerCamera", QVec::vec3(0, 0, 0), "pan").print("pose de lC");
// 	printf("del mundo a la base\n");
// 	model->transform("world", QVec::vec3(0, 0, 0), "base");//.print("pose del robot");
// 	model->transform("world", QVec::vec3(0, 0, 0), "base").print("pose del mundo");

	QVec imgPoint(2);
	imgPoint(0)=200;
	imgPoint(1)=200;

	QVec pPlane = model->projectFromCameraToPlane("leftCamera", imgPoint, "base", QVec::vec3(0,1,0), 200);
	
	pPlane.print("pPlane");

	qFatal("Finished :-)");

	a.exec();
	delete model;
}
