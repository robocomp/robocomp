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
#ifndef DIFFERENTIALROBOTI_H
#define DIFFERENTIALROBOTI_H

// Qt includes
#include <QMutex>
#include <QObject>
#include <QThread>

// RoboComp includes
#include <Ice/Ice.h>
#include <DifferentialRobot.h>
#include <OmniRobot.h>
#include <omnirobotI.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"



using namespace RoboCompDifferentialRobot;
using namespace RoboCompOmniRobot;

class SpecificWorker;

class DifferentialRobotI : public QThread, public virtual RoboCompDifferentialRobot::DifferentialRobot
{
	Q_OBJECT
public:
	DifferentialRobotI(SpecificWorker *_worker, OmniRobotI *_omniI=NULL, QObject *parent = 0);
	~DifferentialRobotI();

	void add(QString id);
	void run();
	void updateInnerModelPose(bool force=false);

	void getBaseState(RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current());
	void getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current& = Ice::Current());
	void setSpeedBase(Ice::Float adv, Ice::Float rot, const Ice::Current& = Ice::Current());
	void stopBase(const Ice::Current& = Ice::Current());
	void resetOdometer(const Ice::Current& = Ice::Current());
	void setOdometer(const RoboCompGenericBase::TBaseState& state, const Ice::Current& = Ice::Current());
	void setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& = Ice::Current());
	void correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& = Ice::Current());

private:
	SpecificWorker *worker;
	InnerModel *innerModel;
	QStringList differentialIDs;
	QMutex *mutex;

	RMat::RTMat zeroTR;
	float zeroANG;

	// Real Noisy Pose
	RoboCompGenericBase::TBaseState pose;
	// Odometry pose
	RoboCompGenericBase::TBaseState noisyPose;
	// Real Angle
	double newAngle;
	//Noisy Angle
	double noisyNewAngle;

	timeval lastCommand_timeval;
	float advVel, rotVel;

	InnerModelTransform *parent;
	InnerModelDifferentialRobot *node;
	InnerModelOmniRobot *nodeOmni;
	OmniRobotI *omniI;
	InnerModelTransform *realNode;

	bool canMoveBaseTo(const QString nodeId, const QVec position, const double alpha);
	void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);

};

#endif
