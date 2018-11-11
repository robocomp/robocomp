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
#ifndef OMNIROBOTI_H
#define OMNIROBOTI_H

// RoboComp includes
#include <Ice/Ice.h>
#include <OmniRobot.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"

using namespace RoboCompOmniRobot;

class SpecificWorker;

class OmniRobotI : public QThread, public virtual RoboCompOmniRobot::OmniRobot
{
	public:
		OmniRobotI ( std::shared_ptr<SpecificWorker> _worker, QObject *parent = 0 );
		
		void add(QString id);
		void run();
		void updateInnerModelPose(bool force=false);
		
		void getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current());
		void getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current& = Ice::Current());
		void setSpeedBase(Ice::Float advx, Ice::Float advz, Ice::Float rot, const Ice::Current& = Ice::Current());
		void stopBase(const Ice::Current& = Ice::Current());
		void resetOdometer(const Ice::Current& = Ice::Current());
		void setOdometer(const RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current());
		void setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& = Ice::Current());
		void correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& = Ice::Current());

	private:
		std::shared_ptr<SpecificWorker> worker;
		std::shared_ptr<InnerModel> innerModel;
		QStringList omniIDs;
		timeval lastCommand_timeval;
		float advVelx, advVelz, rotVel;

		InnerModelTransform *parent;
		InnerModelOmniRobot *node;
		InnerModelTransform *rawOdometryNode, *rawOdometryParentNode;
		InnerModelTransform *correctedOdometryNode, *correctedOdometryParentNode;
		InnerModelTransform *movementFutureNode;
		
		bool canMoveBaseTo(const QString nodeId);
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
};

#endif
