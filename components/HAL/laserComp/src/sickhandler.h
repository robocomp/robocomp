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

#if COMPILE_SICK==1

#ifndef SICKHANDLER_H
#define SICKHANDLER_H

#include <QtCore>

#include <q4serialport/q4serialport.h>
#include <sicklms-1.0/SickLMS.hh>

#include <Laser.h>
#include <DifferentialRobot.h>
#include <const.h>
#include <math.h>
#include "generichandler.h"

using namespace SickToolbox;

class SickLaserHandler : public GenericLaserHandler
{
Q_OBJECT
	SickLMS        *sicklms;
	std::string     sicklms_dev_path;
	sick_lms_baud_t sicklms_desired_baud;
	double          sick_scan_angle;
	double          sick_scan_resolution;
	RoboCompLaser::TLaserData laserData;
	RoboCompLaser::LaserConfData laserConf;
	// Laser data 
	unsigned int polarvalues[SickLMS::SICK_MAX_NUM_MEASUREMENTS]; // Uses macro defined in SickLMS.hh
	unsigned int num_values;
	double       laserPolarData[361];
	double div_factor;
	double scanres;
	sick_lms_measuring_units_t units;
	
public: 
	SickLaserHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent = 0);
	~SickLaserHandler();

	void setConfig(RoboCompLaser::LaserConfData & config){};
	bool open(){return true;};

private:
	void run();
	bool readLaserData();


	RoboCompLaser::TLaserData getNewData();
	RoboCompLaser::LaserConfData getLaserConf();

};
#endif
#endif