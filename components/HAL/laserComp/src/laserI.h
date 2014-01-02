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
#ifndef LASERI_H
#define LASERI_H

#include <string>
#include <iostream>

#include <QtCore>
#include <Laser.h>
#include <DifferentialRobot.h>

#include "generichandler.h"

#include "hokuyohandler.h"
#include "hokuyogenerichandler.h"
#include "gazebohandler.h"
#include "playerhandler.h"
#include "sickhandler.h"

class LaserI : public QObject , public virtual RoboCompLaser::Laser
{
Q_OBJECT
public:
	LaserI(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *parent = 0 );
	~LaserI() {}

private:
	GenericLaserHandler *lh;
	RoboCompDifferentialRobot::TBaseState bState;

public slots:
	RoboCompLaser::TLaserData getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState,const Ice::Current& = ::Ice::Current());
	RoboCompLaser::TLaserData getLaserData( const Ice::Current& = ::Ice::Current());
	RoboCompLaser::LaserConfData getLaserConfData(const Ice::Current& = ::Ice::Current());
};

#endif
