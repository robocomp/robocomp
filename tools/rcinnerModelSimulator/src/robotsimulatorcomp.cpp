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
/** \mainpage RoboComp::genericComp
 *
 * \section intro_sec Introduction
 *
 * The genericComp component...
 *
 * \section interface_sec Interface
 *
 * genericComp interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * genericComp ...
 *
 * \subsection install2_ssec Compile and install
 * cd genericComp
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
 * The configuration file genericComp/etc/config ...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/genericComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */

// Qt includes
#include <QtCore>
#include <QtGui>

// RoboComp includes
#include <Ice/Ice.h>
#include <Ice/Application.h>
#include <IceStorm/IceStorm.h>

// Simulator includes
#include <qlog/qlog.h>
#include <rapplication/rapplication.h>

#include "innermodelmanagerI.h"
#include "config.h"
#include "genericworker.h"
#include "specificworker.h"
#include <RCISMousePicker.h> 
#include "ui_guiDlg.h"

class robotSimulatorComp : public RoboComp::Application
{
private:
	MapPrx mprx;
	void initialize();

public:
	virtual int run( int argc, char* argv[] );
};

void robotSimulatorComp::initialize()
{
}

int robotSimulatorComp::run( int argc, char* argv[] )
{
	// GUI application
	QApplication a(argc, argv);
	initialize();

	printf("---------------------------------------\n");
	printf("--- FCL_SUPPORT: %d\n", InnerModel::support_fcl());
	printf("---------------------------------------\n");

	// Get the port number
	int port = 11175;
	int ms = 30;

	for (int params=2; params+1<argc; params+=2)
	{
		if (strcmp(argv[params],"-p")==0)
		{
			sscanf(argv[params+1], "%d", &port);
		}
		else if (strcmp(argv[params],"-f")==0 )
		{
			sscanf(argv[params+1], "%d", &ms);
		}
	}
	if (port > 65535 or port < 0)
		qFatal("Port number must be in the range 0-1023 (requires root privileges) or 1024-65535 (no privileges needed). Default port number: 11175");
	if (ms > 1000 or ms < 5)
		qFatal("Simulation period must be in the range 5-1000. Default period length 30ms");

	//Talk to STORM and setup mouse publishing stuff
	IceStorm::TopicManagerPrx topicManager;
	IceStorm::TopicPrx rcis_mousepicker_topic;
	Ice::ObjectPrx rcis_mousepicker_pub;
	RoboCompRCISMousePicker::RCISMousePickerPrx rcis_mousepicker;
	RoboCompRCISMousePicker::RCISMousePickerPrx rcis_mousepicker_proxy;
	try
	{ 	
		topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->stringToProxy("IceStorm/TopicManager:tcp -p 9999"));
		int contador = 0;
		while (!rcis_mousepicker_topic and ++contador<50)
		{
			try
			{ rcis_mousepicker_topic = topicManager->retrieve("RCISMousePicker"); }
			catch (const IceStorm::NoSuchTopic&)
			{
				try	{ rcis_mousepicker_topic = topicManager->create("RCISMousePicker");	}
				catch (const IceStorm::TopicExists&){ qDebug() << "Another client already created the topic RCISMousePicker"; }
			}
			rcis_mousepicker_pub = rcis_mousepicker_topic->getPublisher()->ice_oneway();
			rcis_mousepicker = RCISMousePickerPrx::uncheckedCast(rcis_mousepicker_pub);
			mprx["RCISMousePickerPub"] = (::IceProxy::Ice::Object*)(&rcis_mousepicker);
 			rcis_mousepicker_proxy = (*(RCISMousePickerPrx*)mprx["RCISMousePickerPub"]);
 			//qDebug() << "koool0";
		}
		if(contador>=50)
			qFatal("Aborting. Could not connect to STORM although it is alive");
	}
	catch( const Ice::Exception &ex)
	{	
		mprx["RCISMousePickerPub"] = NULL;
		qDebug() << "Warning: STORM not found. RCISMousePicker topic will NOT be published";
	}

	// Create the worker
	//SpecificWorker* worker = new SpecificWorker(mprx, communicator(), argv[1], ms);
	std::shared_ptr<SpecificWorker> worker = std::make_shared<SpecificWorker>(mprx, communicator(), argv[1], ms);
	
	try
	{
		// Server adapter creation and publication
		char endpoint[1024];
		sprintf(endpoint, "tcp -p %d", port);
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapterWithEndpoints("InnerModelSimulator", endpoint);
		InnerModelManagerI *innermodelmanagerI = new InnerModelManagerI(worker);
		adapter->add(innermodelmanagerI, Ice::stringToIdentity("innermodelmanager"));
		adapter->activate();
		cout << SERVER_FULL_NAME " started in port " << port << endl;

		// Start the interfaces
		//worker->startServers();

		// Run the Qt event loop
		a.setQuitOnLastWindowClosed( true );
		a.exec();
		return EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;
		a.quit();
		return EXIT_FAILURE;
	}
}

int main(int argc, char* argv[])
{
// 	bool hasConfig = false;
	string arg;
	robotSimulatorComp app;

	if (argc < 2)
		qFatal("Usage: %s InnerModelFile.xml [-p INNERMODEL_MANAGER_PORT] [-f MSECS]", argv[0]);

	//  0 program_name
	//  1 innermodel
	//  2 optionally -p
	//  3 optionally port
	//  4
	//  5
	//  6
	//  7
	//  8
	//  9
	// 10 0

	// Set arguments
	int argcs = argc + 10;

	// Allog memory and end the arg list
	char *argv2[argcs+1];
	for (int i=0; i<argcs; i++)
		argv2[i] = new char[i<argc?strlen(argv[i])+1:500];
	argv2[argcs] = 0;

	// Copy existing args
	for (int i=0; i<argc; ++i)
		strcpy(argv2[i], argv[i]);

	strcpy( argv2[argc+0], "--Ice.Warn.Connections=0");
	strcpy( argv2[argc+1], "--Ice.Trace.Network=0");
	strcpy( argv2[argc+2], "--Ice.Trace.Protocol=0");
	strcpy( argv2[argc+3], "--Ice.ACM.Client=10");
	strcpy( argv2[argc+4], "--Ice.ACM.Server=10");
	strcpy( argv2[argc+5], "--Ice.MessageSizeMax=20480");
	strcpy( argv2[argc+6], "--Ice.ThreadPool.Client.Size=3");
	strcpy( argv2[argc+7], "--Ice.ThreadPool.Client.SizeMax=15");
	strcpy( argv2[argc+8], "--Ice.ThreadPool.Server.Size=3");
	strcpy( argv2[argc+9], "--Ice.ThreadPool.Server.SizeMax=15");
	
// 	printf("Args:\n");
// 	for (int i=0; i<argcs; i++)
// 		printf(" %s\n", argv2[i]);
	return app.main( argcs, argv2 );
}
