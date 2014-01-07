/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "genericmonitor.h"
/**
* \brief Default constructor
*/
GenericMonitor::GenericMonitor ( GenericWorker *_worker,Ice::CommunicatorPtr _communicator )
{
	worker = _worker;
	this->communicator = _communicator;
	period = 100;
	state = RoboCompCommonBehavior::Starting;
}


/**
* \brief Default destructor
*/
GenericMonitor::~GenericMonitor()
{

}


/**
* \brief Get component execution state
* @return State Component state
*/
RoboCompCommonBehavior::State GenericMonitor::getState()
{
	return state;
}


/**
* \brief Get worker period
* @return int Worker period in ms
*/
int GenericMonitor::getPeriod()
{
	return period;
}


/**
* \brief Change worker period
* @param per Period in ms
*/
void GenericMonitor::setPeriod ( int _period )
{
	worker->setPeriod ( _period );
	period =_period;
}


/**
* \brief Kill component
*/
void GenericMonitor::killYourSelf()
{
	rDebug ( "Killing myself" );
	this->exit();
	worker->killYourSelf();
	emit kill();
}


/**
* \brief Get Component time awake
* @return int Time alive in seconds
*/
int GenericMonitor::timeAwake()
{
	return initialTime.secsTo ( QTime::currentTime() );
}


/**
* \brief Return components parameters
* @return  AttrList Configuration parameters list
*/
RoboCompCommonBehavior::ParameterList GenericMonitor::getParameterList()
{
	return config_params;
}


/**
* \brief Change configurations parameters to worker
* @param l Configuration parameters list
*/
void GenericMonitor::setParameterList ( RoboCompCommonBehavior::ParameterList l )
{
	rInfo ( "Changing configuration params" );
	sendParamsToWorker ( l );
}


//Ice Methods to read from file
bool GenericMonitor::configGetString ( const std::string name, std::string &value,  const std::string default_value, QStringList *list )
{
	value = communicator->getProperties()->getProperty ( name );
	if ( value.length() == 0 ) {
		value = default_value;
		return false;
	}
	std::cout << name << " " << value << std::endl;
	return true;
}
