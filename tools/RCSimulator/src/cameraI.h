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
#ifndef CAMERAI_H
#define CAMERAI_H

// Qt includes
#include <QMutex>
#include <QObject>

// RoboComp includes
#include <Ice/Ice.h>
#include <Camera.h>

// Simulator includes
#include "config.h"
#include "genericworker.h"




using namespace RoboCompCamera;

class CameraI : public QObject , public virtual RoboCompCamera::Camera
{
	Q_OBJECT
public:
	CameraI ( GenericWorker *_worker, QObject *parent = 0 );
	~CameraI();
	
	void getYUVImage ( Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	void getYImage ( Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	void getYLogPolarImage ( Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	void getYImageCR ( Ice::Int cam, Ice::Int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	void getRGBPackedImage ( Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	void getYRGBImage ( Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current& = Ice::Current() );
	TCamParams getCamParams ( const Ice::Current& = Ice::Current() );
	void setInnerImage ( const RoboCompCamera::imgType& roi, const Ice::Current& = Ice::Current() );

private:
	GenericWorker *worker;
	QMutex *mutex;
};

#endif
