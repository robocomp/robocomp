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

LaserI::LaserI ( std::shared_ptr<SpecificWorker> _worker, QObject *parent ) : QObject ( parent )
{
	worker = _worker;
	group = worker->getRootGroup();
}

LaserI::~LaserI()
{
}

void LaserI::add (std::string _id)
{
	id = _id;
	laserNode = worker->innerModel->getNode<InnerModelLaser>(QString::fromStdString(id));
	laserConf.staticConf   =  1;
	laserConf.maxMeasures  =  laserNode->measures;
	laserConf.maxDegrees   = ( int ) ( 180.f*laserNode->angle/M_PI );
	laserConf.maxRange     =  laserNode->max;
	laserConf.minRange     =  laserNode->min;
	laserConf.iniRange     =  -laserConf.maxDegrees/2;
	laserConf.endRange     =  laserConf.maxDegrees/2;
	laserConf.sampleRate   =  30;
	laserConf.angleRes     = ( float ) laserConf.maxDegrees / ( float ) laserConf.maxMeasures;
	laserConf.driver       =  "RCIS";
	laserConf.device       =  id;
	laserConf.angleIni     =  laserConf.iniRange;
}


LaserConfData LaserI::getLaserConfData ( const Ice::Current& )
{
	guard gl(worker->innerModel->mutex);
	return laserConf;
}

TLaserData LaserI::getLaserData ( const Ice::Current& ) //SHOULD CONVERT Laser.Ice to LaserArray.ice
{
	guard gl(worker->innerModel->mutex);
	RoboCompGenericBase::TBaseState state;
	return this->getLaserAndBStateData (state);
}

TLaserData LaserI::getLaserAndBStateData ( RoboCompGenericBase::TBaseState& state, const Ice::Current& )
{
		guard gl(worker->innerModel->mutex);

		IMVLaser &las = worker->imv->lasers[QString::fromStdString(id)];
		QString laserConfig = las.laserNode->ifconfig;
		uint32_t basePort  = laserConfig.toUInt();
	
		//std::map<uint32_t, DifferentialRobotServer>::iterator it = worker->servers.dfr_servers.find( basePort );
		auto it = worker->servers.hMaps.find( basePort );
		//if( it != worker->servers.dfr_servers.end() ) 
		if( it != worker->servers.hMaps.end() ) 
			std::get<DifferentialRobotServer>(it->second).interface->getBaseState ( state );
		
		//if(worker->laserDataArray.contains(QString::fromStdString(id))) 
		if(worker->laserDataArray.count(id)>0) 
		{
//  			for(auto &l : worker->laserDataArray[id])
//  				qDebug() << l.dist << l.angle;
//  			qDebug() << __FILE__ << __FUNCTION__ << ".........................................................................";
			return worker->laserDataArray[id];
		}
		else 
		{
			RoboCompLaser::TLaserData l;
			qDebug() << __LINE__ << "Error unknown Laser " << QString::fromStdString(id);  //SHOULD RETURN A LASER EXCEPTION
			return l;
		}
}
