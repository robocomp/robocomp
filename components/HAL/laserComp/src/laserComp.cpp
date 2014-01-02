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
/** \mainpage RoboComp::laserComp
 *
 * \section intro_sec Introduction
 *
 * The laserComp manages the robot's laser device, processing of data provided by the laser.
 *
 * \section interface_sec Interface
 *
 * laserComp interface returns the configuration data structure and sequence of laser measurements taken (distances and angles).
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * laserComp ....
 *
 * \subsection install2_ssec Compile and install
 * cd Components/HAL/laserComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file laserComp/etc/config you can select different types of driver for laser, initialize the configuration parameters, initialize the connection port... See config.
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/laserComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ....
 *
 */
#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceUtil/IceUtil.h>

#include <QtCore>
#include <QApplication>

#include <DifferentialRobot.h>

// Interface implementation
#include "laserI.h"

#include "const.h"
#include "generichandler.h"

using namespace std;

class LaserComp : public Ice::Application
{
private:
	string laser_device;
	RoboCompLaser::LaserConfData laser_config;
	void initialize();
	bool configGetString( const std::string name, std::string &value, const std::string default_value = "" );
	bool configGetInt( const std::string name, int &value, const int default_value  = 0 );
public:
	virtual int run(int, char*[]);
};

bool LaserComp::configGetString( const std::string name, std::string &value, const std::string default_value )
{
	value = communicator()->getProperties()->getProperty( name );
	if ( value.length() == 0)
	{
		value = default_value;
		return false;
	}

	return true;
}

bool LaserComp::configGetInt( const std::string name, int &value, const int default_value )
{
	string tmp;

	tmp = communicator()->getProperties()->getProperty( name );
	if ( (tmp.length() == 0) || (std::atoi( tmp.c_str() ) == -1) )
	{
		value = default_value;
		return false;
	}

	value = std::atoi( tmp.c_str() );
	return true;
}

void LaserComp::initialize()
{
	// Read configuration values from config file
	configGetString(LASER_DRIVER_PROPERTY_NAME, laser_config.driver, LASER_DRIVER_PROPERTY_DEFAULT);
	configGetString(LASER_DEVICE_PROPERTY_NAME, laser_config.device, LASER_DEVICE_PROPERTY_DEFAULT);
	configGetInt(LASER_START_PROPERTY_NAME, laser_config.iniRange, LASER_START_PROPERTY_DEFAULT);
	configGetInt(LASER_END_PROPERTY_NAME, laser_config.endRange, LASER_END_PROPERTY_DEFAULT);
	configGetInt(LASER_SKIP_PROPERTY_NAME, laser_config.cluster, LASER_SKIP_PROPERTY_DEFAULT);
	configGetInt(LASER_SAMPLERATE_PROPERTY_NAME, laser_config.sampleRate, LASER_SAMPLERATE_PROPERTY_DEFAULT);


	configGetInt("", laser_config.maxDegrees, 240);
	configGetInt("", laser_config.maxRange, 4094);
	configGetInt("", laser_config.minRange, 40);
	laser_config.staticConf = 1;


	laser_config.angleRes = LASER_ANGLE_RESOLUTION_DEFAULT;
	laser_config.angleIni = LASER_INITIAL_ANGLE_DEFAULT;


}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	LaserComp app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}


	if ( hasConfig )
	  return app.main( argc, argv );
	else
	  return app.main(argc, argv, "config");
}

int LaserComp::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);
	int status=EXIT_SUCCESS;

	initialize();

	RoboCompDifferentialRobot::DifferentialRobotPrx base_prx;
	string proxy;

	try
	{
		// Load the remote server proxy
		proxy = communicator()->getProperties()->getProperty( "DifferentialRobotProxy" );
		cout << "[" << PROGRAM_NAME << "]: Loading [" << proxy << "] proxy at '" << "DifferentialRobotProxy" << "'..." << endl;
		if( proxy.empty() )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy config! Check config file for missing of incorrect proxies" << endl;
			return EXIT_FAILURE;
		}

		base_prx = RoboCompDifferentialRobot::DifferentialRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if (!base_prx)
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy!" << endl;
			return EXIT_FAILURE;
		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
		return EXIT_FAILURE;
	}
	cout << "DifferentialRobotProxy initialized Ok!" << endl;

		// Now you can use remote server proxy (remotecomponent_proxy) as local object


	try
	{
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("LaserComp");
		LaserI *laserI = new LaserI(laser_config, base_prx);
		adapter->add(laserI, communicator()->stringToIdentity("laser"));
		adapter->activate();

		cout << "[" PROGRAM_NAME "]: started" << endl;
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		qFatal("OF");
		status = EXIT_FAILURE;
	}

	return status;
}
