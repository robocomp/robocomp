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
#ifdef COMPILE_GAZEBO

#include "gazebohandler.h"

GazeboLaserHandler::GazeboLaserHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent) : GenericLaserHandler(base_prx, _parent)
{
	if ( config.device.size() == 0 )
	{
		qFatal("[GazeboLaserHandler]: Error: Instance created with empty \"device\" configuration.");
	}

	setConfig(config);

#ifdef OLD_API
	client = new gazebo::Client();
	simIface = new gazebo::SimulationIface();
	laserIface = new gazebo::LaserIface();
#else
	client = new libgazebo::Client();
	simIface = new libgazebo::SimulationIface();
	laserIface = new libgazebo::LaserIface();
#endif

	ready = false;
	open();
}

GazeboLaserHandler::~GazeboLaserHandler()
{
 	laserIface->Close();
 	simIface->Close();
}

bool GazeboLaserHandler::open()
{
	if (ready) return true;

	// Connect to the libgazebo server
	client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
	// Open the Simulation Interface
	simIface->Open(client, "default");
  // Open the laser interface
	laserIface->Open(client, confLaser.device.c_str());

	laserIface->Lock(1);
	confLaser.staticConf = 1;
	confLaser.maxMeasures = laserIface->data->range_count;
	confLaser.maxDegrees = laserIface->data->max_angle-laserIface->data->min_angle;
	confLaser.maxRange = laserIface->data->max_range;
 	confLaser.minRange = 2;
	confLaser.iniRange = 0;
	confLaser.endRange = laserIface->data->range_count-1;
	confLaser.cluster = 1;
	laserIface->Unlock();

	ready = true;

	wdataR.resize(confLaser.maxMeasures);
	wdataW.resize(confLaser.maxMeasures);

	return readLaserData();
}

void GazeboLaserHandler::setConfig(RoboCompLaser::LaserConfData &config)
{
	confLaser = config;
}


bool GazeboLaserHandler::readLaserData()
{
	laserIface->Lock(1);
	float maxRange = 0;
	for (int i=0; i<laserIface->data->range_count; ++i)
	{
		wdataW[i].dist = laserIface->data->ranges[i]*1000.;
		wdataW[i].angle = laserIface->data->max_angle - i*(laserIface->data->max_angle-laserIface->data->min_angle)/laserIface->data->range_count;
	}
	laserIface->Unlock();

	//Double buffering
	QMutexLocker mlocker(&laserMutex);
	wdataW.swap(wdataR);
	return true;
}



RoboCompLaser::TLaserData GazeboLaserHandler::getNewData()
{
	QMutexLocker mlocker(&laserMutex);
	return wdataR;
}



RoboCompLaser::LaserConfData GazeboLaserHandler::getLaserConf()
{
	return confLaser;
}

void GazeboLaserHandler::run()
{
	for (;;)
	{
		readLaserData();
		usleep(1000*confLaser.sampleRate);
	}
}

#endif
