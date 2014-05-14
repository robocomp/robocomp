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

// Simulator includes
#include <qlog/qlog.h>
#include <rapplication/rapplication.h>

#include "innermodelmanagerI.h"
#include "config.h"
#include "genericworker.h"
#include "servers.h"
#include "specificworker.h"

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
	int ms = 120;
	for (int params=2; params+1<argc; params+=2)
	{
		if (strcmp(argv[params],"-p")==0)
		{
			sscanf(argv[params+1], "%d", &port);
		}
		else if (strcmp(argv[params],"-f")==0)
		{
			sscanf(argv[params+1], "%d", &ms);
		}
	}
	if (port > 65535 or port < 0)
		qFatal("Port number must be in the range 0-1023 (requires root privileges) or 1024-65535 (no privileges needed). Default port number: 11175");
	if (ms > 1000 or ms < 5)
		qFatal("Simulation period must be in the range 5-1000. Default period length 120ms");
	
	// Create the worker
	SpecificWorker* worker = new SpecificWorker( mprx, communicator(), argv[1], ms );
	try
	{
		// Server adapter creation and publication
		char endpoint[1024];
		sprintf(endpoint, "tcp -p %d", port);
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapterWithEndpoints("InnerModelSimulator", endpoint);
		InnerModelManagerI *innermodelmanagerI = new InnerModelManagerI((SpecificWorker *)worker );
		adapter->add(innermodelmanagerI, communicator()->stringToIdentity("innermodelmanager"));
		adapter->activate();
		cout << SERVER_FULL_NAME " started in port " << port << endl;

		// Start the interfaces
		worker->startServers();

		// Run the Qt event loop
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
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
		qFatal("Usage: %s InnerModelFile.xml", argv[0]);

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
	int argcs = argc + 6;
	
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

	printf("Args:\n");
	for (int i=0; i<argcs; i++)
	printf(" %s\n", argv2[i]);
	return app.main( argcs, argv2 );
}
