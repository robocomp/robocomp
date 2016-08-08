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
#include "commonbehaviorI.h"
/**
* \brief Default constructor
*/
CommonBehaviorI::CommonBehaviorI(GenericMonitor *_monitor, QObject *parent) : QObject(parent)
{
	monitor = _monitor;
	//mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}

/**
* \brief Default destructor
*/
CommonBehaviorI::~CommonBehaviorI()
{
	// Free component resources here
}

// Component functions, implementation
/**
* \brief Return compute period in ms
* @return Compute period in ms
*/
int CommonBehaviorI::getPeriod( const Ice::Current&) 
{ 
	return monitor->getPeriod();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void CommonBehaviorI::setPeriod(int period, const Ice::Current&) 
{
	monitor->setPeriod(period);
}
/**
* \brief Get Component time awake
* @return int Time alive in seconds
*/
int CommonBehaviorI::timeAwake( const Ice::Current&) 
{ 
	return monitor->timeAwake();
}
/**
* \brief Kill component
*/
void CommonBehaviorI::killYourSelf( const Ice::Current&) 
{
	monitor->killYourSelf();
}
/**
* \brief Return components parameters
* @return  AttrList Configuration parameters list
*/
ParameterList CommonBehaviorI::getParameterList( const Ice::Current&) 
{ 
	return monitor->getParameterList();
}
/**
* \brief Change configurations parameters to worker
* @param l Configuration parameters list
*/
void CommonBehaviorI::setParameterList(const RoboCompCommonBehavior::ParameterList &l, const Ice::Current&) 
{ 
	monitor->setParameterList(l);
}
void CommonBehaviorI::reloadConfig( const Ice::Current&)
{
	//monitor->readConfig();
}
/**
* \brief Get component execution state
* @return State Component state
*/
RoboCompCommonBehavior::State CommonBehaviorI::getState(const Ice::Current&)
{
	return monitor->getState();
}