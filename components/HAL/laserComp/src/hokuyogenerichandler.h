/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#ifdef COMPILE_HOKUYO30LX

#ifndef HOKUYOGENERICHANDLER_H
#define HOKUYOGENERICHANDLER_H

#include <generichandler.h>
#include <QtCore>
#include <Laser.h>
#include <const.h>
#include <math.h>



extern "C"
{
#include <c_urg/urg_ctrl.h>
}

#define ERROR_C 20.
#define LIMITE 440.

class HokuyoGenericHandler : public GenericLaserHandler
{
Q_OBJECT
public:
	HokuyoGenericHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent = 0);
	~HokuyoGenericHandler();
	void setConfig(RoboCompLaser::LaserConfData & config);
	bool open();
	bool isPowerOn();

private:

	urg_t urg;
	long *data;
	int data_max;
	RoboCompLaser::TLaserData wdataR, wdataW;
	RoboCompLaser::LaserConfData confLaser;
	QTimer *timer,*pm_timer;
	QTime last_use;
	bool powerOn;
	QMutex laserMutex;
	void run();

	bool setLaserPowerState(bool);
	bool readLaserData();
	int SiguienteNoNulo(RoboCompLaser::TLaserData & laserData,int pos,int aStart,int aEnd);

private slots:
	bool poweronLaser();
	void poweroffLaser();
	void checkLaserUse();

public slots:
	RoboCompLaser::TLaserData getNewData();
	RoboCompLaser::LaserConfData getLaserConf();
};

#endif

#endif
