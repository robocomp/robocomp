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
#include "omnirobotI.h"
#include "specificworker.h"

OmniRobotI::OmniRobotI(SpecificWorker *_worker, QObject *parent): QThread(parent)
{
	worker = _worker;
	mutex = worker->mutex;
	innerModel = worker->getInnerModel();
	advVelx = advVelz = rotVel = 0;
	gettimeofday(&lastCommand_timeval, NULL);
	updateInnerModelPose();

	zeroANG = 0;
	zeroTR = RTMat(0,0,0, 0,0,0);
}


void OmniRobotI::add(QString id)
{
	omniIDs << id;
	node = innerModel->getOmniRobot(id);
	parent = innerModel->getTransform(node->parent->id);
	newAngle = innerModel->getRotationMatrixTo(parent->id, id).extractAnglesR()(1);
	noisyNewAngle = innerModel->getRotationMatrixTo(parent->id, id).extractAnglesR()(1);
	realNode = innerModel->newTransform(id+"_odometry\"", "static", parent, 0, 0, 0, 0, newAngle, 0);
}


void OmniRobotI::run()
{
	updateInnerModelPose(true);
	while (true)
	{
		mutex->lock();
		updateInnerModelPose();
		mutex->unlock();
		usleep(10000);
	}
}


OmniRobotI::~OmniRobotI()
{
	// Free component resources here
}


// Component functions, implementation
void OmniRobotI::getBaseState(RoboCompOmniRobot::TBaseState& state, const Ice::Current&)
{
	QMutexLocker locker(mutex);
	
	state = pose;
	QVec retPOS = (zeroTR * QVec::vec3(pose.x, 0, pose.z).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	state.x = retPOS(0);
	state.z = retPOS(2);
	state.alpha = pose.alpha - zeroANG;
	
	retPOS = (zeroTR * QVec::vec3(pose.correctedX, 0, pose.correctedZ).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	state.correctedX = retPOS(0);
	state.correctedZ = retPOS(2);
	state.correctedAlpha = pose.correctedAlpha - zeroANG;
}


void OmniRobotI::getBasePose(Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current&)
{
	QMutexLocker locker(mutex);
	QVec retPOS = (zeroTR * QVec::vec3(pose.x, 0, pose.z).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	x = retPOS(0);
	z = retPOS(2);
	alpha = pose.alpha - zeroANG;
}


#define MILIMETERS_PER_UNIT 1.
void OmniRobotI::updateInnerModelPose(bool force)
{
	if ( (fabs(advVelx)<0.0001 and fabs(advVelz)<0.0001 and fabs(rotVel)<0.0001) and not force)
	{
		return;
	}

	timeval now;
	gettimeofday(&now, NULL);
	const double msecs = (now.tv_sec - lastCommand_timeval.tv_sec)*1000. +(now.tv_usec - lastCommand_timeval.tv_usec)/1000.;
	lastCommand_timeval = now;

	QVec vel = QVec::vec3(advVelx, 0, advVelz);
	QVec newPos, noisyNewPos;
	const double noise = node->noise;
	
	// Random noise:
	QVec rndmPos = QVec::gaussianSamples(2, 0, noise*(0.01*vel.norm2() + 0.1*rotVel));
	QVec rndmYaw = QVec::gaussianSamples(1, 0, noise*(0.01*vel.norm2() + 0.1*rotVel));

	// Without noise
	QVec T = vel.operator*(msecs / 1000.);
	float Angle  = rotVel*msecs / 1000.;
	newAngle += Angle;
	
	QVec backNoisyNewPos = innerModel->transform(parent->id, QVec::vec3(0,0,0), node->id);
	float backNoisyAngle = noisyNewAngle;
	noisyNewAngle += Angle + rndmYaw[0];
	noisyNewPos = innerModel->transform(parent->id, QVec::vec3(T(0), 0, T(2)), node->id);
	innerModel->updateTransformValues(node->id, noisyNewPos(0), noisyNewPos(1), noisyNewPos(2), 0, noisyNewAngle, 0);
	if (canMoveBaseTo(node->id, noisyNewPos, noisyNewAngle+Angle+(rndmYaw[0]*noise) ))
	{
		// Noisy pose(real)
		pose.x     = noisyPose.x     = noisyNewPos(0)*MILIMETERS_PER_UNIT;
		pose.z     = noisyPose.z     = noisyNewPos(2)*MILIMETERS_PER_UNIT;
		pose.alpha = noisyPose.alpha = noisyNewAngle;
	}
	else
	{
		noisyNewAngle = backNoisyAngle;
		noisyNewPos = backNoisyNewPos;
		innerModel->updateTransformValues(node->id, noisyNewPos(0), noisyNewPos(1), noisyNewPos(2), 0, noisyNewAngle, 0);
	}
	newPos = innerModel->transform(parent->id, QVec::vec3(T(0)+rndmPos[0], 0, T(2)+rndmPos[1]), node->id+"_odometry\"");
	innerModel->updateTransformValues(node->id+"_odometry\"", newPos(0), newPos(1), newPos(2), 0, newAngle, 0);

	// Pose without noise (as if I moved perfectly)
	pose.correctedX = newPos(0)*MILIMETERS_PER_UNIT;
	pose.correctedZ = newPos(2)*MILIMETERS_PER_UNIT;
	pose.correctedAlpha =  newAngle;
}

bool OmniRobotI::canMoveBaseTo(const QString nodeId, const QVec position, const double alpha)
{
	std::vector<QString> robotNodes;
	std::vector<QString> restNodes;

	recursiveIncludeMeshes(innerModel->getRoot(), nodeId, false, robotNodes, restNodes);

	for (uint32_t in=0; in<robotNodes.size(); in++)
	{
		for (uint32_t out=0; out<restNodes.size(); out++)
		{
			if (innerModel->collide(robotNodes[in], restNodes[out]))
			{
				return false;
			}
		}
	}

	return true;
}

void OmniRobotI::recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out)
{
	if (node->id == robotId)
	{
		inside = true;
	}
	
	InnerModelMesh *mesh;
	InnerModelPlane *plane;
	InnerModelTransform *transformation;

	if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
	{
		for (int i=0; i<node->children.size(); i++)
		{
			recursiveIncludeMeshes(node->children[i], robotId, inside, in, out);
		}
	}
	else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
	{
		if (inside)
		{
			in.push_back(node->id);
		}
		else
		{
			out.push_back(node->id);
		}
	}
}


void OmniRobotI::setSpeedBase(Ice::Float advx, Ice::Float advz, Ice::Float rot, const Ice::Current&)
{
	QMutexLocker locker(mutex);
	updateInnerModelPose();
	gettimeofday(&lastCommand_timeval, NULL);
	advVelx = advx;
	advVelz = advz;
	rotVel = rot;
	pose.advVx = advx;
	pose.advVz = advz;
	pose.rotV = rot;
}


void OmniRobotI::stopBase(const Ice::Current&)
{
	setSpeedBase(0.,0.,0.);
}


void OmniRobotI::resetOdometer(const Ice::Current&)
{
	QMutexLocker locker(mutex);
	zeroANG = pose.alpha;
	zeroTR = RTMat(0, -pose.alpha, 0, 0, 0, 0)* RTMat(0, 0, 0, -pose.x, 0, -pose.z);
}


void OmniRobotI::setOdometer(const RoboCompOmniRobot::TBaseState& st, const Ice::Current&)
{
	setOdometerPose(st.x, st.z, st.alpha);
}


void OmniRobotI::setOdometerPose(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	QMutexLocker locker(mutex);
	zeroANG = pose.alpha-alpha;
	zeroTR = RTMat(0,0,0,  x,0,z)*RTMat(0,alpha-pose.alpha,0, 0,0,0)* RTMat(0,0,0, -pose.x,0,-pose.z);
}


void OmniRobotI::correctOdometer(Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current&)
{
	QMutexLocker locker(mutex);
	pose.correctedX = x;
	pose.correctedZ = z;
	pose.correctedAlpha = alpha;
}

