/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
#include "agmcommonbehaviorI.h"

AGMCommonBehaviorI::AGMCommonBehaviorI(GenericWorker *_worker)
{
	worker = _worker;
}


AGMCommonBehaviorI::~AGMCommonBehaviorI()
{
}


bool AGMCommonBehaviorI::activateAgent(const RoboCompAGMCommonBehavior::ParameterMap &prs, const Ice::Current&)
{
	return worker->AGMCommonBehavior_activateAgent(prs);
}

bool AGMCommonBehaviorI::deactivateAgent(const Ice::Current&)
{
	return worker->AGMCommonBehavior_deactivateAgent();
}

RoboCompAGMCommonBehavior::ParameterMap AGMCommonBehaviorI::getAgentParameters(const Ice::Current&)
{
	return worker->AGMCommonBehavior_getAgentParameters();
}

RoboCompAGMCommonBehavior::StateStruct AGMCommonBehaviorI::getAgentState(const Ice::Current&)
{
	return worker->AGMCommonBehavior_getAgentState();
}

void AGMCommonBehaviorI::killAgent(const Ice::Current&)
{
	worker->AGMCommonBehavior_killAgent();
}

bool AGMCommonBehaviorI::reloadConfigAgent(const Ice::Current&)
{
	return worker->AGMCommonBehavior_reloadConfigAgent();
}

bool AGMCommonBehaviorI::setAgentParameters(const RoboCompAGMCommonBehavior::ParameterMap &prs, const Ice::Current&)
{
	return worker->AGMCommonBehavior_setAgentParameters(prs);
}

int AGMCommonBehaviorI::uptimeAgent(const Ice::Current&)
{
	return worker->AGMCommonBehavior_uptimeAgent();
}

