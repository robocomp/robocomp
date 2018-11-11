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
#include "touchsensorI.h"
#include "specificworker.h"
#include <math.h>

/**
* \brief Default constructor
*/
TouchSensorI::TouchSensorI ( std::shared_ptr<SpecificWorker> _worker, QObject *parent ) : QObject ( parent )
{}

/**
* \brief Default destructor
*/
TouchSensorI::~TouchSensorI()
{
	// Free component resources here
}

void TouchSensorI::add(std::string id)
{
	sensorIDs.push_back(id);
	SensorState state;
	state.value = 0;
	sensorMap[id] = state;
}

void TouchSensorI::remove(std::string id)
{
	sensorIDs.erase(std::remove(sensorIDs.begin(), sensorIDs.end(), id), sensorIDs.end());
	sensorMap.erase(id);
}

RoboCompTouchSensor::SensorMap TouchSensorI::getValues(const Ice::Current&)
{
	return sensorMap;
}

