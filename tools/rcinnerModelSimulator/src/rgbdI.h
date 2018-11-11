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
#ifndef RGBDI_H
#define RGBDI_H

// RoboComp includes
#include <Ice/Ice.h>
#include <RGBD.h>
#include <innermodel/innermodel.h>

// Simulator includes
#include "config.h"

using namespace RoboCompRGBD;

class SpecificWorker;

class RGBDI : public QObject , public virtual RoboCompRGBD::RGBD
{
	public:
		RGBDI ( std::shared_ptr<SpecificWorker> _worker, QObject* parent = 0 );
		~RGBDI();
		void add ( QString id );
		TRGBDParams getRGBDParams ( const Ice::Current & = Ice::Current() );
		void setRegistration ( Registration value, const Ice::Current & = Ice::Current() );
		Registration getRegistration ( const Ice::Current & = Ice::Current() );
		void getData ( RoboCompRGBD::imgType& rgbMatrix, depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		void getDepthInIR ( depthType& distanceMatrix, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		void getImage ( ColorSeq& color, DepthSeq& depth, PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		void getDepth ( DepthSeq& depth, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		void getRGB ( ColorSeq& color, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		void getXYZ ( PointSeq& points, RoboCompJointMotor::MotorStateMap& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current & = Ice::Current() );
		
	private:
		std::shared_ptr<SpecificWorker> worker;
		QString id;
};

#endif
