/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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

/**
	\brief
	@author authorname
*/

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool AGMCommonBehavior_activateAgent(const RoboCompAGMCommonBehavior::ParameterMap &prs);
	bool AGMCommonBehavior_deactivateAgent();
	RoboCompAGMCommonBehavior::ParameterMap AGMCommonBehavior_getAgentParameters();
	RoboCompAGMCommonBehavior::StateStruct AGMCommonBehavior_getAgentState();
	void AGMCommonBehavior_killAgent();
	bool AGMCommonBehavior_reloadConfigAgent();
	bool AGMCommonBehavior_setAgentParameters(const RoboCompAGMCommonBehavior::ParameterMap &prs);
	int AGMCommonBehavior_uptimeAgent();

	void AGMExecutiveTopic_edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void AGMExecutiveTopic_edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void AGMExecutiveTopic_selfEdgeAdded(const int nodeid, const std::string &edgeType, const RoboCompAGMWorldModel::StringDictionary &attributes);
	void AGMExecutiveTopic_selfEdgeDeleted(const int nodeid, const std::string &edgeType);
	void AGMExecutiveTopic_structuralChange(const RoboCompAGMWorldModel::World &w);
	void AGMExecutiveTopic_symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void AGMExecutiveTopic_symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);

public slots:
	int startup_check();
	void initialize(int period);
	//Specification slot methods State Machine
	void sm_publish();
	void sm_pop_data();
	void sm_read_uwb();
	void sm_read_rs();
	void sm_read_april();
	void sm_compute_pose();
	void sm_initialize();
	void sm_finalize();

	//--------------------
private:
	std::shared_ptr < InnerModel > innerModel;
	std::string action;
	RoboCompAGMCommonBehavior::ParameterMap params;
	AGMModel::SPtr worldModel;
	bool active;
	bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);
	bool startup_check_flag;

};

#endif
