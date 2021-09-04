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
#ifndef SPECIFICMONITOR_H
#define SPECIFICMONITOR_H

#include "genericmonitor.h"

/**
       \brief
       @author authorname
*/
class SpecificMonitor : public GenericMonitor
{
  Q_OBJECT
  
  public:
	SpecificMonitor(GenericWorker *_worker, Ice::CommunicatorPtr _communicator);
	~SpecificMonitor();
	
	void readConfig(RoboCompCommonBehavior::ParameterList &params );
	void run();
	void initialize();
    
	bool sendParamsToWorker(RoboCompCommonBehavior::ParameterList params);
	bool checkParams(RoboCompCommonBehavior::ParameterList l);
	
	bool ready;
};

#endif // GENERICMONITOR_H
