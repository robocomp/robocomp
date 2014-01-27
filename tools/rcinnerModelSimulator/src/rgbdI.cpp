/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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
#include "rgbdI.h"
#include "specificworker.h"

/**
* \brief Default constructor
*/
RGBDI::RGBDI ( SpecificWorker *_worker, QObject *parent ) : QObject ( parent )
{
	worker = _worker;
}

/**
* \brief Default destructor
*/
RGBDI::~RGBDI()
{
}

void RGBDI::add ( QString _id )
{
	id = _id;
}


TRGBDParams RGBDI::getRGBDParams ( const Ice::Current& )
{
	return worker->rgbd_getRGBDParams ( id );
}


void RGBDI::setRegistration ( Registration value, const Ice::Current& )
{
	worker->rgbd_setRegistration ( id, value );
}


Registration RGBDI::getRegistration ( const Ice::Current& )
{
	return worker->rgbd_getRegistration( id );
}


void RGBDI::getData ( RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	DepthSeq depth;
	PointSeq points;
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
	
	rgbMatrix.resize ( 640*480*3 );
	distanceMatrix.resize ( 640*480 );
	for ( int i=0; i<640*480; i++ ) {
		rgbMatrix[3*i+0] = color[i].red;
		rgbMatrix[3*i+1] = color[i].green;
		rgbMatrix[3*i+2] = color[i].blue;
		distanceMatrix[i] = depth[i];
	}
}


void RGBDI::getDepthInIR ( depthType& distanceMatrix, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	DepthSeq depth;
	PointSeq points;
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
	
	distanceMatrix.resize ( 640*480 );
	for ( int i=0; i<640*480; i++ ) {
		distanceMatrix[i] = depth[i];
	}
}


void RGBDI::getImage ( ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
}


void RGBDI::getDepth ( DepthSeq& depth, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	PointSeq points;
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
}


void RGBDI::getRGB ( ColorSeq& color, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	DepthSeq depth;
	PointSeq points;
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
}


void RGBDI::getXYZ ( PointSeq& points, RoboCompJointMotor ::MotorStateMap& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& )
{
	ColorSeq color;
	DepthSeq depth;
	worker->rgbd_getImage ( id, color, depth, points, hState, bState );
}

