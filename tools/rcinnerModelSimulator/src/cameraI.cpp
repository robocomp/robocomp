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
#include "cameraI.h"

CameraI::CameraI(GenericWorker *_worker, QObject *parent ) : QObject(parent )
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


CameraI::~CameraI()
{
	// Free component resources here
}


// Component functions, implementation
void CameraI::getYUVImage(Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


void CameraI::getYImage(Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


void CameraI::getYLogPolarImage(Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


void CameraI::getYImageCR(Ice::Int cam, Ice::Int div, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


void CameraI::getRGBPackedImage(Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


void CameraI::getYRGBImage(Ice::Int cam, RoboCompCamera::imgType& roi, RoboCompCommonHead::THeadState& hState, RoboCompGenericBase::TBaseState& bState, const Ice::Current& )
{
}


TCamParams CameraI::getCamParams(const Ice::Current& )
{
	TCamParams p;
	return p;
}


void CameraI::setInnerImage(const RoboCompCamera::imgType& roi, const Ice::Current& )
{
}


