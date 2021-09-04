/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "laserI.h"

LaserI::LaserI(GenericWorker *_worker)
{
	worker = _worker;
}


LaserI::~LaserI()
{
}


RoboCompLaser::TLaserData LaserI::getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState, const Ice::Current&)
{
	return worker->Laser_getLaserAndBStateData(bState);
}

RoboCompLaser::LaserConfData LaserI::getLaserConfData(const Ice::Current&)
{
	return worker->Laser_getLaserConfData();
}

RoboCompLaser::TLaserData LaserI::getLaserData(const Ice::Current&)
{
	return worker->Laser_getLaserData();
}

