/*
 *    Copyright (C) 2009-2010 by RoboLab - University of Extremadura
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
#ifdef COMPILE_PLAYER

#ifndef PLAYERLASERHANDLER_H
#define PLAYERLASERHANDLER_H

#include "generichandler.h"
#include <vector>
#include <QtCore>

#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif
// #include <gazebo/gazebo.h>
///lib player
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;
namespace boost
{
  namespace signalslib = signals;
}
#if defined(signals) && defined(QOBJECTDEFS_H) && !defined(QT_MOC_CPP)
#  undef signals
#  define signals protected
#endif

class PlayerLaserHandler : public GenericLaserHandler
{
public:
	PlayerLaserHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent = 0);
	~PlayerLaserHandler();

private:
    PlayerClient *client;
	LaserProxy *laserIface;
	QMutex *mutex;
	
// 	gazebo::Client *client;
// 	gazebo::SimulationIface *simIface;
// 	gazebo::LaserIface *laserIface;
	bool ready;
	RoboCompLaser::TLaserData wdataR, wdataW;
	RoboCompLaser::LaserConfData confLaser;
	RoboCompDifferentialRobot::DifferentialRobotPrx base;

public:
	void setConfig(RoboCompLaser::LaserConfData & config);
	bool open();
	QMutex laserMutex;

private:
	void run();

	bool readLaserData();
	RoboCompLaser::TLaserData getNewData() ;
	RoboCompLaser::LaserConfData getLaserConf();
};


#endif
#endif


