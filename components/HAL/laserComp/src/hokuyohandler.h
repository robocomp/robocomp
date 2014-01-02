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
#ifndef LASERHANDLER_H
#define LASERHANDLER_H

#include <QtCore>

#include <q4serialport/q4serialport.h>

#include <Laser.h>
#include <const.h>
#include <math.h>

#include "generichandler.h"

#define ERROR_C 20.
#define ERROR_L 4000.
#define LIMITE 440.

class HokuyoHandler : public GenericLaserHandler
{
Q_OBJECT
public:
	HokuyoHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent = 0);
	~HokuyoHandler();

	void setConfig(RoboCompLaser::LaserConfData & config);
	bool open();
	bool isPowerOn();

private:
	QSerialPort laserDevice;
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
