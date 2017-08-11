/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
#ifndef GENERICBASEI_H
#define GENERICBASEI_H

// Qt includes
#include <QMutex>
#include <QObject>
#include <QThread>

// RoboComp includes
#include <Ice/Ice.h>
#include <GenericBase.h>
#include <innermodel/innermodel.h>
#include "innermodelmgr.h"   /// Cambiar cuando este en la lib!!!

// Simulator includes
#include "config.h"



using namespace RoboCompGenericBase;

class SpecificWorker;

class GenericBaseI : public QObject, public virtual RoboCompGenericBase::GenericBase
{
Q_OBJECT
public:
	GenericBaseI ( SpecificWorker *_worker, QObject *parent = 0 );
	
	void add(QString id);
	//void run();
	
	void getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current & =Ice::Current());
	void getBasePose(int &x, int &z, float &alpha, const Ice::Current & =Ice::Current());

private:
	SpecificWorker *worker;
	InnerModelMgr innerModel;
	InnerModelTransform *parent;
	InnerModelOmniRobot *node;
};

#endif







