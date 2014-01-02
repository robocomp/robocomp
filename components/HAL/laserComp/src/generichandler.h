/*
 *    Copyright (C) 2009-2010 by RoboLab - University of Extremadura
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
#ifndef GENERIC_LASER_HANDLER_H
#define GENERIC_LASER_HANDLER_H

#include <stdint.h>
#include <QtCore>
#include <iostream>
#include <Laser.h>

#include <q4serialport/q4serialport.h>

#include <const.h>
#include <math.h>


/**
	\class GenericLaserHandler <p>Generic abstract class for LaserComp component handlers. It defines a generic interface for laser handler classes. Real laser handlers should be implemented as a derived class.</p>
 */
class GenericLaserHandler : public QThread
{
Q_OBJECT
public:
	struct HandlerConfigType
	{
		int start;
		int end;
		int skip;
		int samplerate;
	};
	GenericLaserHandler(RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent = 0) : QThread(_parent) {
		base = base_prx;
	}
	~GenericLaserHandler() {}

	virtual void setConfig(RoboCompLaser::LaserConfData & config) = 0;
	virtual bool open() = 0;
	RoboCompDifferentialRobot::DifferentialRobotPrx base;
private:
	virtual void run() = 0;
	virtual bool readLaserData() = 0;
	

public slots:
	RoboCompDifferentialRobot::TBaseState getBaseState()
	{
		RoboCompDifferentialRobot::TBaseState b;
		try{
			base->getBaseState(b);
		}
		catch(Ice::Exception e)
		{
			qDebug()<<"error talking to differentialRobot";
		}
		return b;
	}

	virtual RoboCompLaser::TLaserData getNewData() = 0;
	virtual RoboCompLaser::LaserConfData getLaserConf() = 0;
};

#endif
