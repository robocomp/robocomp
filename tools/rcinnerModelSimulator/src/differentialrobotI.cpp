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
#include "differentialrobotI.h"
#include "specificworker.h"

DifferentialRobotI::DifferentialRobotI ( SpecificWorker *_worker, QObject *parent ) : QThread ( parent )
{
	worker = _worker;
	mutex = worker->mutex;
	innerModel = worker->getInnerModel();
	advVel = rotVel = 0;
	gettimeofday ( &lastCommand_timeval, NULL );
	updateInnerModelPose();

	zeroANG = 0;
	zeroTR = RTMat(0,0,0, 0,0,0);
}


void DifferentialRobotI::add ( QString id )
{
	differentialIDs << id;
	node = innerModel->getDifferentialRobot ( id );
	parent = innerModel->getTransform ( node->parent->id );
	newAngle = innerModel->getRotationMatrixTo ( parent->id, id ).extractAnglesR() ( 1 );
	noisyNewAngle = innerModel->getRotationMatrixTo ( parent->id, id ).extractAnglesR() ( 1 );
	realNode = innerModel->newTransform ( id+"_odometry\"", "static", parent, 0, 0, 0, 0, newAngle, 0 );
}


void DifferentialRobotI::run()
{
	updateInnerModelPose(true);
	while (true)
	{
		mutex->lock();
		updateInnerModelPose();
		mutex->unlock();
		usleep ( 10000 );
	}
}


DifferentialRobotI::~DifferentialRobotI()
{
	// Free component resources here
}


// Component functions, implementation
void DifferentialRobotI::getBaseState ( TBaseState& state, const Ice::Current& )
{
	QMutexLocker locker ( mutex );
	
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


void DifferentialRobotI::getBasePose ( Ice::Int& x, Ice::Int& z, Ice::Float& alpha, const Ice::Current& )
{
	QMutexLocker locker ( mutex );
	QVec retPOS = (zeroTR * QVec::vec3(pose.x, 0, pose.z).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	x = retPOS(0);
	z = retPOS(2);
	alpha = pose.alpha - zeroANG;
}


#define MILIMETERS_PER_UNIT 1.
void DifferentialRobotI::updateInnerModelPose ( bool force )
{
	if ( ( fabs ( advVel ) < 0.00000001 and fabs ( rotVel ) < 0.00000001 ) and not force )
	{
		return;
	}

	timeval now;
	gettimeofday ( &now, NULL );
	const double msecs = ( now.tv_sec - lastCommand_timeval.tv_sec ) *1000. + ( now.tv_usec - lastCommand_timeval.tv_usec ) /1000.;
	lastCommand_timeval = now;
	QVec newPos, noisyNewPos;
	double noise = 0.001;
	//With random noise:
	QVec rndmRot = QVec::gaussianSamples ( 1, 0, 0.01*noise );
	QVec rndmAdv = QVec::gaussianSamples ( 1, 0, 0.01*noise );
	QVec rndmYaw = QVec::gaussianSamples ( 1, 0, 0.01*noise );


	double Ax1, Az1;
	double Ax2, Az2;
	if ( fabs ( rotVel ) < 0.00000001 )
	{
		// With noise:
		Ax1 = ( advVel*rndmRot[0]*noise*0.1 ) *msecs / 1000.;
		Az1 = ( advVel+ ( rndmAdv[0]*noise ) ) *msecs / ( 1000. * MILIMETERS_PER_UNIT );
		noisyNewAngle += rndmYaw[0]*noise;
		// Without noise:
		Ax2 = 0;
		Az2 = advVel * msecs / ( 1000. * MILIMETERS_PER_UNIT );
	}
	else
	{
		// With noise:
		double T = advVel*msecs / 1000.;
		double Angle = msecs * rotVel / 1000.;
		Ax1 = ( ( 1-cos ( Angle ) ) /Angle ) *T* ( 1.+ ( rndmRot[0]*noise ) );
		Az1 = ( sin ( Angle ) /Angle ) *T* ( 1.+ ( rndmAdv[0]*noise ) );
		noisyNewAngle += Angle+ ( ( rndmYaw[0]*noise ) );
		// Without noise
		T = advVel*msecs / 1000.;
		Angle = msecs * rotVel / 1000.;
		Ax2 = ( ( 1-cos ( Angle ) ) /Angle ) *T;
		Az2 = ( sin ( Angle ) /Angle ) *T;
		newAngle += Angle;
	}
	noisyNewPos = innerModel->transform ( parent->id, QVec::vec3 ( Ax1, 0, Az1 ), node->id );
	innerModel->updateTransformValues ( node->id, noisyNewPos ( 0 ), noisyNewPos ( 1 ), noisyNewPos ( 2 ), 0, noisyNewAngle, 0 );
	newPos = innerModel->transform ( parent->id, QVec::vec3 ( Ax2, 0, Az2 ), node->id+"_odometry\"" );
	innerModel->updateTransformValues ( node->id+"_odometry\"", newPos ( 0 ), newPos ( 1 ), newPos ( 2 ), 0, newAngle, 0 );

	// Pose without noise (as if I moved perfectly)
	pose.x = newPos ( 0 ) *MILIMETERS_PER_UNIT;
	pose.z = newPos ( 2 ) *MILIMETERS_PER_UNIT;
	pose.alpha = newAngle;

	// noisy pose (real)
	pose.correctedX = noisyPose.x = noisyNewPos(0)*MILIMETERS_PER_UNIT;
	pose.correctedZ = noisyPose.z = noisyNewPos(2)*MILIMETERS_PER_UNIT;
	pose.correctedAlpha = noisyPose.alpha = noisyNewAngle;
}


void DifferentialRobotI::setSpeedBase ( Ice::Float adv, Ice::Float rot, const Ice::Current& )
{
	QMutexLocker locker ( mutex );
	updateInnerModelPose();
	gettimeofday ( &lastCommand_timeval, NULL );
	advVel = adv;
	rotVel = rot;
	pose.advV = adv;
	pose.rotV = rot;
}


void DifferentialRobotI::stopBase ( const Ice::Current& )
{
	setSpeedBase(0,0);
}


void DifferentialRobotI::resetOdometer ( const Ice::Current& )
{
	QMutexLocker locker(mutex);
	zeroANG = pose.alpha;
	zeroTR = RTMat(0, -pose.alpha, 0, 0, 0, 0) * RTMat(0, 0, 0, -pose.x, 0, -pose.z);
}


void DifferentialRobotI::setOdometer ( const TBaseState& st, const Ice::Current& )
{
	setOdometerPose(st.x, st.z, st.alpha);
}


void DifferentialRobotI::setOdometerPose ( Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& )
{
	QMutexLocker locker ( mutex );
	zeroANG = pose.alpha-alpha;
	zeroTR = RTMat(0,0,0,  x,0,z) *RTMat(0,alpha-pose.alpha,0, 0,0,0) * RTMat(0,0,0, -pose.x,0,-pose.z);
}


void DifferentialRobotI::correctOdometer ( Ice::Int x, Ice::Int z, Ice::Float alpha, const Ice::Current& )
{
	QMutexLocker locker ( mutex );
	pose.correctedX = x;
	pose.correctedZ = z;
	pose.correctedAlpha = alpha;
}

