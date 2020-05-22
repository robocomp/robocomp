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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(MapPrx& mprx) : QObject()
{

	//Initialization State machine
	initializeState = new QState(QState::ExclusiveStates);
	customMachine.addState(initializeState);
	publishState = new QState(QState::ExclusiveStates);
	customMachine.addState(publishState);
	pop_dataState = new QState(QState::ExclusiveStates);
	customMachine.addState(pop_dataState);
	read_uwbState = new QState(QState::ExclusiveStates);
	customMachine.addState(read_uwbState);
	read_rsState = new QState(QState::ExclusiveStates);
	customMachine.addState(read_rsState);
	read_aprilState = new QState(QState::ExclusiveStates);
	customMachine.addState(read_aprilState);
	compute_poseState = new QState(QState::ExclusiveStates);
	customMachine.addState(compute_poseState);
	finalizeState = new QFinalState();
	customMachine.addState(finalizeState);

	customMachine.setInitialState(initializeState);

	initializeState->addTransition(this, SIGNAL(t_initialize_to_pop_data()), pop_dataState);
	pop_dataState->addTransition(this, SIGNAL(t_pop_data_to_pop_data()), pop_dataState);
	pop_dataState->addTransition(this, SIGNAL(t_pop_data_to_read_uwb()), read_uwbState);
	read_uwbState->addTransition(this, SIGNAL(t_read_uwb_to_pop_data()), pop_dataState);
	pop_dataState->addTransition(this, SIGNAL(t_pop_data_to_read_rs()), read_rsState);
	pop_dataState->addTransition(this, SIGNAL(t_pop_data_to_read_april()), read_aprilState);
	read_aprilState->addTransition(this, SIGNAL(t_read_april_to_compute_pose()), compute_poseState);
	read_rsState->addTransition(this, SIGNAL(t_read_rs_to_compute_pose()), compute_poseState);
	compute_poseState->addTransition(this, SIGNAL(t_compute_pose_to_publish()), publishState);
	compute_poseState->addTransition(this, SIGNAL(t_compute_pose_to_pop_data()), pop_dataState);
	publishState->addTransition(this, SIGNAL(t_publish_to_pop_data()), pop_dataState);
	pop_dataState->addTransition(this, SIGNAL(t_pop_data_to_finalize()), finalizeState);

	QObject::connect(initializeState, SIGNAL(entered()), this, SLOT(sm_initialize()));
	QObject::connect(publishState, SIGNAL(entered()), this, SLOT(sm_publish()));
	QObject::connect(pop_dataState, SIGNAL(entered()), this, SLOT(sm_pop_data()));
	QObject::connect(read_uwbState, SIGNAL(entered()), this, SLOT(sm_read_uwb()));
	QObject::connect(read_rsState, SIGNAL(entered()), this, SLOT(sm_read_rs()));
	QObject::connect(read_aprilState, SIGNAL(entered()), this, SLOT(sm_read_april()));
	QObject::connect(compute_poseState, SIGNAL(entered()), this, SLOT(sm_compute_pose()));
	QObject::connect(finalizeState, SIGNAL(entered()), this, SLOT(sm_finalize()));

	//------------------
	agmexecutive_proxy = (*(AGMExecutivePrx*)mprx["AGMExecutiveProxy"]);

	mutex = new QMutex(QMutex::Recursive);

	Period = BASIC_PERIOD;

}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}

RoboCompPlanning::Action GenericWorker::createAction(std::string s)
{
	// Remove useless characters
	char chars[]="()";
		for (unsigned int i=0; i<strlen(chars); ++i)
	{
		s.erase(std::remove(s.begin(), s.end(), chars[i]), s.end());
	}

		// Initialize string parsing
	RoboCompPlanning::Action ret;
	istringstream iss(s);

	// Get action (first segment)
	if (not iss)
	{
		printf("agent %s: received invalid action (%s) -> (%d)\n", PROGRAM_NAME, __FILE__, __LINE__);
		exit(-1);
	}
	else
	{
		iss >> ret.name;
	}

	do
	{
		std::string ss;
		iss >> ss;
		ret.symbols.push_back(ss);
	} while (iss);

	return ret;
}

bool GenericWorker::activate(const BehaviorParameters &prs)
{
	printf("Worker::activate\n");
	mutex->lock();
	p = prs;
	active = true;
	iter = 0;
	mutex->unlock();
	return active;
}

bool GenericWorker::deactivate()
{
	printf("Worker::deactivate\n");
	mutex->lock();
	active = false;
	iter = 0;
	mutex->unlock();
	return active;
}

bool GenericWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		// Action
		p.action = createAction(params["action"].value);

		// Fill received plan
		p.plan.clear();
		QStringList actionList = QString::fromStdString(params["plan"].value).split(QRegExp("[()]+"), QString::SkipEmptyParts);
		for (int32_t actionString=0; actionString<actionList.size(); actionString++)
		{
			std::vector<string> elementsVec;
			QStringList elements = actionList[actionString].remove(QChar('\n')).split(QRegExp("\\s+"), QString::SkipEmptyParts);
			for (int32_t elem=0; elem<elements.size(); elem++)
			{
				elementsVec.push_back(elements[elem].toStdString());
			}
			p.plan.push_back(elementsVec);
		}
	}
	catch (...)
	{
		return false;
	}

	// Check if we should reactivate the component
	if (isActive())
	{
		activate(p);
		reactivated = true;
	}

	return true;
}
