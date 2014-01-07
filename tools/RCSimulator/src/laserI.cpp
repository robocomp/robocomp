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
#include "laserI.h"

#include "specificworker.h"

LaserI::LaserI ( SpecificWorker *_worker, QObject *parent ) : QObject ( parent )
{
	worker = _worker;
	mutex = worker->mutex;
	innerModel = worker->getInnerModel();
}


LaserI::~LaserI()
{

}


void LaserI::add ( QString _id )
{
	id = _id;
	laserNode = innerModel->getLaser ( id );
	laserConf.staticConf   =  1;
	laserConf.maxMeasures  =  laserNode->im_measures;
	laserConf.maxDegrees   = ( int ) ( 180.f*laserNode->im_angle/M_PI );
	laserConf.maxRange     =  laserNode->im_max;
	laserConf.minRange     =  laserNode->im_min;
	laserConf.iniRange     =  -laserConf.maxDegrees/2;
	laserConf.endRange     =  laserConf.maxDegrees/2;
	laserConf.sampleRate   =  30;
	laserConf.angleRes     = ( float ) laserConf.maxDegrees / ( float ) laserConf.maxMeasures;
	laserConf.driver       =  "RCIS";
	laserConf.device       =  id.toStdString();
	laserConf.angleIni     =  laserConf.iniRange;
}


LaserConfData LaserI::getLaserConfData ( const Ice::Current& )
{
	return laserConf;
}


TLaserData LaserI::getLaserData ( const Ice::Current& ) //SHOULD CONVERT Laser.Ice to LaserArray.ice
{
	RoboCompDifferentialRobot::TBaseState state;
	return worker->laser_getLaserAndBStateData ( id, state );
}


TLaserData LaserI::getLaserAndBStateData ( RoboCompDifferentialRobot::TBaseState& state, const Ice::Current& )
{
	return worker->laser_getLaserAndBStateData ( id, state );
}
