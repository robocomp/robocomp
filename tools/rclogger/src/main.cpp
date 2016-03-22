/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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


/** \mainpage RoboComp::rclogger
 *
 * \section intro_sec Introduction
 *
 * The rclogger component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd rclogger
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
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/rclogger --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <loggerI.h>

#include <Logger.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

using namespace RoboCompLogger;



class rclogger : public RoboComp::Application
{
public:
	rclogger (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void ::rclogger::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::rclogger::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;


	string proxy, tmp;
	initialize();



	SpecificWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;
	
	while (!monitor->ready)
	{
		usleep(10000);
	}
	
	try
	{
		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
		}
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();




		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "Logger.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Logger";
		}
		Ice::ObjectAdapterPtr adapterLogger = communicator()->createObjectAdapterWithEndpoints("Logger", tmp);
		LoggerI *logger = new LoggerI(worker);
		adapterLogger->add(logger, communicator()->stringToIdentity("logger"));
		adapterLogger->activate();
		cout << "[" << PROGRAM_NAME << "]: Logger adapter created in port " << tmp << endl;





		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
		a.setQuitOnLastWindowClosed( true );
#endif
		// Run QT Application Event Loop
		a.exec();
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
		monitor->exit(0);
}

	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
	::rclogger app(prefix);

	return app.main(argc, argv, configFile.c_str());
}

