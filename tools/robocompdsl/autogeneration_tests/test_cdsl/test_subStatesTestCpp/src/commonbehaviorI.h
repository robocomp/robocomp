/*
 *    Copyright (C) 2020 by RoboLab - University of Extremadura
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
#ifndef COMMONBEHAVIORI_H
#define COMMONBEHAVIORI_H

// Ice includes
#include <Ice/Ice.h>
#include <CommonBehavior.h>

#include <config.h>

#include "genericworker.h"
#include "genericmonitor.h"

/**
	\class CommonBehaviorI <p>Servant for components common behaviors. This class implements the methods of the public interface of CommonBehavior.
*/
class CommonBehaviorI : public virtual RoboCompCommonBehavior::CommonBehavior
{
public:
	CommonBehaviorI( GenericMonitor *_monitor );

	int getPeriod( const Ice::Current & = Ice::Current());
	void setPeriod(int period, const Ice::Current & = Ice::Current());
	int timeAwake( const Ice::Current & = Ice::Current());
	void killYourSelf( const Ice::Current & = Ice::Current());
	RoboCompCommonBehavior::ParameterList getParameterList( const Ice::Current & = Ice::Current());
	void setParameterList(const RoboCompCommonBehavior::ParameterList &l, const Ice::Current & = Ice::Current());
	void reloadConfig( const Ice::Current& = Ice::Current());
	RoboCompCommonBehavior::State getState(const Ice::Current& = Ice::Current());

private:
	GenericMonitor *monitor;	///*<monitor Pointer to access monitor methods. It's used to set or read component configuration.

};

#endif
