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
#ifndef LASERI_H
#define LASERI_H

// Qt includes
#include <QMutex>
#include <QObject>

// RoboComp includes
#include <Ice/Ice.h>
#include <Laser.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"



using namespace RoboCompLaser;

class SpecificWorker;

class LaserI : public QObject , public virtual RoboCompLaser::Laser
{
	Q_OBJECT
public:
	LaserI ( SpecificWorker *_worker, QObject *parent = 0 );
	~LaserI();
	
	void add ( QString id );
	
	TLaserData getLaserData ( const Ice::Current& = Ice::Current() );
	TLaserData getLaserAndBStateData ( RoboCompDifferentialRobot::TBaseState& state, const Ice::Current& = Ice::Current() );
	LaserConfData getLaserConfData ( const Ice::Current& = Ice::Current() );

private:
	QString id;
	IM2::InnerModel *innerModel;
	IM2::Laser *laserNode;
	SpecificWorker *worker;
	LaserConfData laserConf;
	QMutex *mutex;
};

#endif
