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
#include "laserI.h"


LaserI::LaserI(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *parent) : QObject(parent)
{
  	// Component initialization ...
	if ((config.driver == "HokuyoURG")) // HOKUYO URG04LX
	{
	  lh = new HokuyoHandler(config, base_prx);
	}
	else if ((config.driver == "Hokuyo30LX") || (config.driver == "HokuyoURG04LX-UG01"))
	{
		#ifdef COMPILE_HOKUYO30LX
		 lh = new HokuyoGenericHandler(config, base_prx);
		#else
		 qFatal("LaserComp::LaserI::LaserI(): Config error: laserComp was not compiled with HOKUYO Laser support (please, install hokuyo library (sited in robocomp/Thirdparty). Exiting...");
		#endif
	}
	else if (config.driver == "Gazebo")
	{
		#ifdef COMPILE_GAZEBO
		lh = new GazeboLaserHandler(config, base_prx);
		#else
		qFatal("LaserComp::LaserI::LaserI(): Config error: laserComp was not compiled with Gazebo support. Exiting...");
		#endif
	}
	else if (config.driver == "Sick")
	{
		#ifdef COMPILE_SICK
		lh = new SickLaserHandler(config, base_prx);
		#else
		qFatal("LaserComp::LaserI::LaserI(): Config error: laserComp was not compiled with sick support. Exiting...");
		#endif
	}
	else if (config.driver == "Player")
	{
		#ifdef COMPILE_PLAYER
		lh = new PlayerLaserHandler(config, base_prx);
		#else
		qFatal("LaserComp::LaserI::LaserI(): Config error: laserComp was not compiled with Player support. Exiting...");
		#endif
	}
	else
		qFatal( "LaserComp::LaserI::LaserI(): No driver %s available. Aborting ", config.driver.c_str() );

	// lh->setConfig( config );

	if ( !lh->open() )
	{
		qFatal( "[" PROGRAM_NAME "]: Unable to open device: %s", config.device.c_str() );
	}

	qWarning( "[" PROGRAM_NAME "]: Device opened: %s", config.device.c_str() );
	lh->start();
}

RoboCompLaser::TLaserData LaserI::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &bState,const Ice::Current& )
{
	bState = lh->getBaseState();
	return lh->getNewData();
}


RoboCompLaser::TLaserData LaserI::getLaserData(const Ice::Current &)
{
/*	RoboCompLaser::TLaserData data = lh->getNewData();
	std::sort(data.begin(), data.end(), TLaserDataSortPredicate);
	return data;*/
//	qDebug() << "antes " << bState.x << bState.z << bState.alpha;
	return lh->getNewData();
}

RoboCompLaser::LaserConfData LaserI::getLaserConfData(const Ice::Current &)
{
	return lh->getLaserConf();
}

