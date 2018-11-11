/****************************************************************************
#                                                                           #
#                     Copyright (C) 2011                                    #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

#ifndef IMUI_H
#define IMUI_H

// RoboComp includes
#include <Ice/Ice.h>
#include <IMU.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"

////////////////////
//  NEEDS TO BE VECTORIZED

using namespace RoboCompIMU;

class SpecificWorker;

class IMUI : public QObject , public virtual RoboCompIMU::IMU
{
public:
	IMUI ( std::shared_ptr<SpecificWorker> _worker, QObject *parent = 0 );
	~IMUI();
	
	void add ( QString id );
	void updateIMUData ( QString id );
	DataImu getDataImu ( const Ice::Current& = ::Ice::Current() );
	Acceleration getAcceleration ( const Ice::Current& = ::Ice::Current() );
	Gyroscope getAngularVel ( const Ice::Current& = ::Ice::Current() );
	Magnetic getMagneticFields ( const Ice::Current& = ::Ice::Current() );
	Orientation getOrientation ( const Ice::Current& = ::Ice::Current() );
	void resetImu ( const Ice::Current& = ::Ice::Current() );
	
private:
	std::shared_ptr<SpecificWorker> worker;
	QStringList imuIDs;
	DataImu data_imu;
};

#endif
