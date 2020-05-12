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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <QStateMachine>
#include <QState>
#include <CommonBehavior.h>

#include <AGMCommonBehavior.h>
#include <AGMExecutive.h>
#include <AGMExecutiveTopic.h>
#include <AGMWorldModel.h>
#include <Planning.h>
#include <agm.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
using namespace RoboCompAGMCommonBehavior;
using namespace RoboCompAGMExecutive;
using namespace RoboCompAGMExecutiveTopic;
using namespace RoboCompAGMWorldModel;
using namespace RoboCompPlanning;

typedef map <string,::IceProxy::Ice::Object*> MapPrx;


struct BehaviorParameters
{
	RoboCompPlanning::Action action;
	std::vector< std::vector <std::string> > plan;
};

class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	bool activate(const BehaviorParameters& parameters);
	bool deactivate();
	bool isActive() { return active; }


	AGMExecutivePrx agmexecutive_proxy;

	virtual bool AGMCommonBehavior_activateAgent(const ParameterMap &prs) = 0;
	virtual bool AGMCommonBehavior_deactivateAgent() = 0;
	virtual ParameterMap AGMCommonBehavior_getAgentParameters() = 0;
	virtual StateStruct AGMCommonBehavior_getAgentState() = 0;
	virtual void AGMCommonBehavior_killAgent() = 0;
	virtual bool AGMCommonBehavior_reloadConfigAgent() = 0;
	virtual bool AGMCommonBehavior_setAgentParameters(const ParameterMap &prs) = 0;
	virtual int AGMCommonBehavior_uptimeAgent() = 0;
	virtual void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification) = 0;
	virtual void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications) = 0;
	virtual void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes) = 0;
	virtual void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const string &edgeType) = 0;
	virtual void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w) = 0;
	virtual void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification) = 0;
	virtual void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications) = 0;

protected:
	//State Machine
	QStateMachine customMachine;

	QState *publishState;
	QState *pop_dataState;
	QState *read_uwbState;
	QState *read_rsState;
	QState *read_aprilState;
	QState *compute_poseState;
	QState *initializeState;
	QFinalState *finalizeState;

	//-------------------------

	QTimer timer;
	int Period;

	bool active;
	AGMModel::SPtr worldModel;
	BehaviorParameters p;
	ParameterMap params;
	int iter;
	bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
	RoboCompPlanning::Action createAction(std::string s);

private:


public slots:
	//Slots funtion State Machine
	virtual void sm_publish() = 0;
	virtual void sm_pop_data() = 0;
	virtual void sm_read_uwb() = 0;
	virtual void sm_read_rs() = 0;
	virtual void sm_read_april() = 0;
	virtual void sm_compute_pose() = 0;
	virtual void sm_initialize() = 0;
	virtual void sm_finalize() = 0;

	//-------------------------
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
	//Signals for State Machine
	void t_initialize_to_pop_data();
	void t_pop_data_to_pop_data();
	void t_pop_data_to_read_uwb();
	void t_read_uwb_to_pop_data();
	void t_pop_data_to_read_rs();
	void t_pop_data_to_read_april();
	void t_read_april_to_compute_pose();
	void t_read_rs_to_compute_pose();
	void t_compute_pose_to_publish();
	void t_compute_pose_to_pop_data();
	void t_publish_to_pop_data();
	void t_pop_data_to_finalize();

	//-------------------------
};

#endif
