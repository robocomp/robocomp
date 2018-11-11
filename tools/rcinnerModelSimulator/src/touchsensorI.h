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
#ifndef TOUCHSENSORI_H
#define TOUCHSENSORI_H

// RoboComp includes
#include <Ice/Ice.h>
#include <TouchSensor.h>
#include <innermodel/innermodel.h>
#include <string>
#include <vector>

// Simulator includes
#include "config.h"

class SpecificWorker;

using namespace RoboCompTouchSensor;

class TouchSensorI : public QObject , public virtual RoboCompTouchSensor::TouchSensor
{
	public:
		TouchSensorI(std::shared_ptr<SpecificWorker> _worker, QObject *parent = 0);
		~TouchSensorI();
		void add (std::string id );
		void remove(std::string id);
		RoboCompTouchSensor::SensorMap getValues(const Ice::Current&);
	private:
		std::vector<std::string> sensorIDs;
		SensorMap sensorMap;
};

#endif
