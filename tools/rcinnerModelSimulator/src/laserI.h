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

// RoboComp includes
#include <Ice/Ice.h>
#include <Laser.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"
#include <osg/Group>

using namespace RoboCompLaser;
class SpecificWorker;

class LaserI : public QObject , public virtual RoboCompLaser::Laser
{
	public:
		LaserI (std::shared_ptr<SpecificWorker> _worker, QObject *parent = 0 );
		~LaserI();
		
		void add ( std::string id );
		TLaserData getLaserData ( const Ice::Current& = Ice::Current() );
		TLaserData getLaserAndBStateData ( RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current() );
		LaserConfData getLaserConfData ( const Ice::Current& = Ice::Current() );

	private:
		std::string id;
		InnerModelLaser *laserNode;
		osg::Group *group;
		std::shared_ptr<SpecificWorker> worker;
		LaserConfData laserConf;
};

#endif
