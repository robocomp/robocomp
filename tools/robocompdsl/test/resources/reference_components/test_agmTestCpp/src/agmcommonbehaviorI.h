/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
#ifndef AGMCOMMONBEHAVIOR_H
#define AGMCOMMONBEHAVIOR_H

// Ice includes
#include <Ice/Ice.h>
#include <AGMCommonBehavior.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompAGMCommonBehavior;

class AGMCommonBehaviorI : public virtual RoboCompAGMCommonBehavior::AGMCommonBehavior
{
public:
AGMCommonBehaviorI(GenericWorker *_worker);
	~AGMCommonBehaviorI();

	bool activateAgent(const ParameterMap  &prs, const Ice::Current&);
	bool deactivateAgent(const Ice::Current&);
	ParameterMap getAgentParameters(const Ice::Current&);
	StateStruct getAgentState(const Ice::Current&);
	void killAgent(const Ice::Current&);
	bool reloadConfigAgent(const Ice::Current&);
	bool setAgentParameters(const ParameterMap  &prs, const Ice::Current&);
	int uptimeAgent(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
