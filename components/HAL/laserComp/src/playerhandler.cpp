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

#include "playerhandler.h"

PlayerLaserHandler::PlayerLaserHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent) : GenericLaserHandler(base_prx, _parent)
{
   base= base_prx;
  
   if ( config.device.size() == 0 )
	{
		qFatal("[PlayerLaserHandler]: Error: Instance created with empty \"device\" configuration.");
	}

   setConfig(config);
   QString s=config.device.c_str();	
   qDebug()<<s;
	/// Connect to the player server
   client = new PlayerClient(s.split(":")[0].toStdString(),s.split(":")[1].toUInt());
   /// Connect laser
   laserIface = new LaserProxy (client,0);
   mutex = new QMutex();
	ready = false;
	open();
}

PlayerLaserHandler::~PlayerLaserHandler()
{
//  	laserIface->Close();
//  	simIface->Close();  
   
}

bool PlayerLaserHandler::open()
{
	if (ready) return true;
	
// 	laserIface->Lock(1);
    mutex->lock();	
	laserIface->RequestConfigure();
	laserIface->RequestGeom();
	client->Read();
	
	confLaser.staticConf = 1;
// 	confLaser.maxMeasures = laserIface->data->range_count;
    confLaser.maxMeasures = laserIface->GetCount();	
// 	confLaser.maxDegrees = laserIface->data->max_angle-laserIface->data->min_angle;    
    confLaser.maxDegrees = rtod(laserIface->GetConfMaxAngle()) - rtod(laserIface->GetConfMinAngle());
		
 	confLaser.minRange = 20;
	confLaser.maxRange = 4096;
	
// 	confLaser.endRange = laserIface->data->range_count-1;
	confLaser.endRange = confLaser.maxMeasures-1;
	confLaser.iniRange = 0;
// 	in  radian
	confLaser.angleRes = laserIface->GetScanRes();
	confLaser.angleIni = laserIface->GetConfMinAngle();
	
	confLaser.cluster = 1;
	qDebug()<< "******* HokuyoURG laser configuration for Player/Stage *********";	
 	qDebug()<< "driver = " <<confLaser.driver.c_str();
	qDebug()<< "device = " <<confLaser.device.c_str();
	qDebug()<< "staticConf = " <<confLaser.staticConf;
    qDebug()<< "maxMeasures = " <<confLaser.maxMeasures;
 	qDebug()<< "maxDegrees = " <<confLaser.maxDegrees;
 	qDebug()<< "maxRange in mm = " <<confLaser.maxRange;
 	qDebug()<< "minRange in mm = " <<confLaser.minRange;
 	qDebug()<< "iniRange = " <<confLaser.iniRange;
 	qDebug()<< "endRange = " <<confLaser.endRange;
 	qDebug()<< "cluster = " <<confLaser.cluster;
 	qDebug()<< "sampleRate = " <<confLaser.sampleRate;
	qDebug()<< "angleRes in radian = " <<confLaser.angleRes;
	qDebug()<< "angleIni in radian = " <<confLaser.angleIni;
	qDebug()<< "********  end configuration **********"<<endl;
	mutex->unlock();

	ready = true;

	wdataR.resize(confLaser.maxMeasures);
	wdataW.resize(confLaser.maxMeasures);
	return readLaserData();
}

void PlayerLaserHandler::setConfig(RoboCompLaser::LaserConfData &config)
{    
	confLaser = config;
}
// // 
// // 
bool PlayerLaserHandler::readLaserData()
{
// 	laserIface->Lock(1);
	mutex->lock();		
	client->Read();
// 	for (int i=0; i<laserIface->data->range_count; ++i)
	for (int i=0; i<confLaser.maxMeasures; ++i)
	{
// 		wdataW[i].dist = laserIface->data->ranges[i]*1000.;
// 		wdataW[i].angle = laserIface->data->max_angle - i*(laserIface->data->max_angle-laserIface->data->min_angle)/laserIface->data->range_count;
		wdataW[i].dist = laserIface->GetRange(i)*1000.;
		wdataW[i].angle = laserIface->GetMaxAngle() - i*(laserIface->GetMaxAngle()-laserIface->GetMinAngle())/laserIface->GetCount();
// 		if (i==0 or i==laserIface->GetCount()/2 or i==confLaser.endRange)
// 		qDebug()<<i<<" "<<rtod(wdataW[i].angle)<<" "<<wdataW[i].dist<<" "<< laserIface->GetMaxAngle()<<" "<<laserIface->GetMinAngle();
	}
// 	laserIface->Unlock();	
	mutex->unlock();

	//Double buffering
	QMutexLocker mlocker(&laserMutex);
	wdataW.swap(wdataR);
	
	return true;
}
// 
// 
// 
RoboCompLaser::TLaserData PlayerLaserHandler::getNewData()
{
	QMutexLocker mlocker(&laserMutex);
	return wdataR;
}
// 

RoboCompLaser::LaserConfData PlayerLaserHandler::getLaserConf()
{
	return confLaser;
}
// 
void PlayerLaserHandler::run()
{
    
	for (;;)
	{
		readLaserData();		
		usleep(confLaser.sampleRate);
	}
}

#endif
