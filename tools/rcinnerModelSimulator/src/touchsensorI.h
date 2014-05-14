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
#ifndef TOUCHSENSORI_H
#define TOUCHSENSORI_H

// Qt includes
#include <QMutex>
#include <QObject>

// RoboComp includes
#include <Ice/Ice.h>
#include <TouchSensor.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"



class SpecificWorker;

using namespace std;
using namespace RoboCompTouchSensor;



class TouchSensorI : public QObject , public virtual RoboCompTouchSensor::TouchSensor
{
Q_OBJECT
public:
	TouchSensorI(SpecificWorker *_worker, QObject *parent = 0);
	~TouchSensorI();

	void add (QString id );
	void remove(QString id);

	RoboCompTouchSensor::SensorMap getValues(const Ice::Current&);

// private:
	SpecificWorker *worker;
	InnerModel *innerModel;
	QStringList sensorIDs;
	
	SensorMap sensorMap;
	QMutex *mutex;
};

#endif
