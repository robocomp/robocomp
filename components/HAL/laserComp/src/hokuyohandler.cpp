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
#include "hokuyohandler.h"

HokuyoHandler::HokuyoHandler(RoboCompLaser::LaserConfData &config, RoboCompDifferentialRobot::DifferentialRobotPrx base_prx, QObject *_parent) : GenericLaserHandler (base_prx, _parent)
{
	if (config.device.size() == 0)
	{
		qDebug() << "LaserComp::HokuyoHandler::HokuyoHandler(): Warning: Laser device configuration is empty! Trying to guess...";
		// Setting up lase
		QFile file0( "/dev/ttyACM0");
		if (file0.exists())
		{
			config.device = "/dev/ttyACM0";
			qDebug() << "LaserComp::HokuyoHandler::HokuyoHandler(): Lets try with" << QString::fromStdString(config.device);
		}
		else
		{
			QFile file1("/dev/ttyACM1");
			if (file1.exists())
			{
				config.device = "/dev/ttyACM1";
				qDebug() << "LaserComp::HokuyoHandler::HokuyoHandler(): Lets try with" << QString::fromStdString(config.device);
			}
			else
			{
				qFatal("LaserComp::HokuyoHandler::HokuyoHandler(): Error: Laser device configuration is empty. It could not be detected automatically.");
			}
		}
	}

	laserDevice.setName(config.device.c_str());

	// Create the sampling timer
	timer = new QTimer(this );
	pm_timer = NULL;
	powerOn = FALSE;

	setConfig(config);
}

HokuyoHandler::~HokuyoHandler()
{
	timer->stop();
	disconnect ( timer );
	delete timer;
	pm_timer->stop();
	disconnect ( pm_timer );
	delete pm_timer;
}

void HokuyoHandler::setConfig (RoboCompLaser::LaserConfData & config )
{
	confLaser = config;
	int nPoints = confLaser.endRange - confLaser.iniRange + 1;
	wdataR.resize( nPoints );
	wdataW.resize( nPoints );

	// Power management
	if (pm_timer==NULL)
	{
		pm_timer = new QTimer ( this );
		connect ( pm_timer, SIGNAL ( timeout() ), this, SLOT ( checkLaserUse() ) );
		pm_timer->start ( LASER_TIMEOUT );
	}
	if (config.sampleRate != 0)
		timer->setInterval ( 1000 / config.sampleRate );
	else
		qFatal("Sample rate can't be zero.");
}

bool HokuyoHandler::open()
{
	// Open and initialize the device
	if ( !laserDevice.open ( 1 /*QIODevice::ReadWrite*/ ) )
	{
		timer->start();
		qWarning("[HokuyoHandler]: Failed to open: %s", laserDevice.name().toLatin1().data());
		return FALSE;
	}
	for (int i=0; i< 5; i++)
	{
		qDebug() << " Trying to power the Laser device. Try: " << i;
		if (poweronLaser()==TRUE)
			break;
		usleep(300000);
	}
	if (powerOn == FALSE) qFatal("[HokuyoHandler]: Could not start Laser device");

	timer->start();

	return TRUE;
}

bool HokuyoHandler::isPowerOn()
{
	return powerOn;
}

bool HokuyoHandler::poweronLaser()
{
	if ( !powerOn )
	{
		if ( setLaserPowerState ( TRUE ) )
		{
			powerOn = TRUE;
			qWarning ( "[" PROGRAM_NAME "]: Power magnament: laser on." );
			return TRUE;
		}
		else
		{
			qWarning ( "[" PROGRAM_NAME "]: Power magnament: unable to power on laser." );
			return FALSE;
		}
	}
	return true;
}

void HokuyoHandler::poweroffLaser()
{
	if ( powerOn )
	{
		if ( setLaserPowerState ( FALSE ) )
		{
			powerOn = FALSE;
			qWarning ( "[" PROGRAM_NAME "]: Power magnament: laser off." );
		}
		else
		{
			qWarning ( "[" PROGRAM_NAME "]: Power magnament: unable to power off laser." );
		}
	}
}

bool HokuyoHandler::setLaserPowerState ( bool state )
{
	char *command;
	static char data_block[6]; //Two LF's  at end
	int dataWritten, dataRead;

	bzero ( data_block, 5 );
	if ( state )
	{
		command = (char *)LASER_CMD_POWER_ON;
	}
	else
	{
		command = (char *)LASER_CMD_POWER_OFF;
	}


	// Send the command to the laser
	dataWritten = laserDevice.write ( command, LASER_CMD_POWER_SZ );
	if ( !dataWritten )
	{

		qWarning ( "[HokuyoHandler]: Write error on laser device!" );
		return FALSE;
	}

	// Get the echo from the laser
	dataRead = laserDevice.readLine ( data_block, sizeof ( data_block ) );
	if ( strcmp ( command, data_block ) != 0 )
	{
		qWarning ( "[HokuyoHandler]: Laser CRC error setting power status!" );
		return FALSE;
	}

	// Get the laser status ( 0 = ok )
	dataRead = laserDevice.readLine ( data_block, sizeof ( data_block ) );
	if ( strcmp ( "0\n", data_block ) !=0 )
	{
		qWarning ( "[HokuyoHandler]: Error: Laser status != 0" );
		return FALSE;
	}

	// Read the final /n
	dataRead = laserDevice.readLine ( data_block, sizeof ( data_block ) );

	return TRUE;
}

bool HokuyoHandler::readLaserData()
{
	char command[15];
	char data_block[80];
	int dataWritten, dataRead;

	bzero(command, 12);
	bzero(data_block, sizeof(data_block));

	if (powerOn==false)
	{
		if (setLaserPowerState(TRUE) == false)
		{
			return false;
		}
		else
		{
			qWarning ( "[" PROGRAM_NAME "]: Power magnament: laser on." );
		}
	}

	// Create the command: 'G' + Starting point (3) + End Point (3) + Cluster Count (2) + '\n'  SACAR FUERA!!!
	sprintf(command, "G%03d%03d%02d\n", confLaser.iniRange, confLaser.endRange, confLaser.cluster);

	// Send the command to the laser
	dataWritten = laserDevice.write(command, LASER_CMD_GET_DATA_SZ );
	if (!dataWritten)
	{
		qWarning ( "[HokuyoHandler]: Write error on laser device!" );
		return false;
	}

	// Get the echo from the laser
	dataRead = laserDevice.readLine(data_block, sizeof(data_block));
	if (strcmp(command, data_block) != 0)
	{
		qWarning ( "[HokuyoHandler]: Laser CRC error while getting data!" );
		return false;
	}

	// Get the laser status ( 0 = ok )
	dataRead = laserDevice.readLine(data_block, sizeof(data_block));
	if (strcmp("0\n", data_block))
	{
		qWarning("[HokuyoHandler]: Error: Laser status != 0");
		return false;
	}

	int j = 0;  // Decoded data vector indexing
	for ( ;; )
	{
		// Read data from laser
		dataRead = laserDevice.readLine(data_block, sizeof(data_block));
		if ( !dataRead )
		{
		  qFatal ( "[HokuyoHandler]: Error: Unable to read data from laser!" );
		}

		// Empty line means end of data
		if (!strcmp("\n", data_block))
			break;

		// Read one data Block ( max. 64 bytes + '\n' )
		// Data is 12 bits divided in two bytes
		// The first byte has the 6 High bits, and the second byte has the rest
		// Controller add 0x30 to each byte, so we must revert the encoded byte

		for (int i=0; i<(int)strlen(data_block)-1; i++, ++j)
		{
		  wdataW[j].dist = ( ( data_block[i] - 0x30 ) << 6 ) + ( data_block[i + 1] - 0x30 );
			i++;
		}
	}

	float ra;
	int aux;
	int k;
	//	std::cout << "data_received " << wdataW.size() << std::endl;
	for( k=0; k<(int)wdataW.size(); k++)
	{

		ra = wdataW.at(k).dist;
		wdataW[k].angle = -(confLaser.angleIni + (confLaser.iniRange + k)*confLaser.angleRes);

		if ( (ra<ERROR_C) || (ra>ERROR_L) )
		{
			aux = SiguienteNoNulo ( wdataW, k, 0, wdataW.size());
			wdataW[k].dist = 0; //(short)aux;
			// std::cout << "error measurements, dist, angle: " << ra << ", " << wdataW[k].angle*180/M_PI << std::endl;
		}

	}
	powerOn = TRUE;
	//	qFatal("OK");


	//Double buffering
	laserMutex.lock();
	wdataW.swap(wdataR);
	laserMutex.unlock();

	return true;
}

int HokuyoHandler::SiguienteNoNulo(RoboCompLaser::TLaserData & laserData,int pos,int aStart,int aEnd)
{
	int i=1;
	float temp=0.;
	int posaux=0;
	while ( (((pos-i)>aStart)||((pos+i)<aEnd)) && ((temp<ERROR_C)||(temp>ERROR_L)) )
	{
		if ((pos-i)>aStart)
			temp=laserData.at(pos-i).dist;
		if (((pos+i)<aEnd) && ((temp<ERROR_C)||(temp>ERROR_L)))
			temp=laserData.at(pos+i).dist;
		i++;
	}
	i--;
	if ( (temp>=ERROR_C) && (temp<=ERROR_L) )
	{
		if ((pos-i)>=aStart)
		{
			if ( (laserData.at(pos-i).dist>=ERROR_C) && (laserData.at(pos-i).dist<=ERROR_L) )
				posaux=pos-i;
			else
				posaux=pos+i;
		}
	}
	pos=posaux;
	return ( temp );
}


RoboCompLaser::TLaserData HokuyoHandler::getNewData()
{
	QMutexLocker mlocker(&laserMutex);

	return wdataR;
}

RoboCompLaser::LaserConfData HokuyoHandler::getLaserConf()
{
	return confLaser;
}


///Check Laser use for powersaving
void HokuyoHandler::checkLaserUse()
{
	if ( last_use.elapsed() > LASER_TIMEOUT )
		poweroffLaser();
}
///Thread main loop
void HokuyoHandler::run()
{
	for (;;)
	{
		last_use.restart();
		if (readLaserData()==false)
		{
			std::cout << "Error reading laser " << std::endl;
		}

	}
}


