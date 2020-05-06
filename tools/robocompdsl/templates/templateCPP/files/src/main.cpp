/*
[[[cog

import sys
sys.path.append('/opt/robocomp/python')

import cog
def A():
	cog.out('<@@<')
def Z():
	cog.out('>@@>')
def TAB():
	cog.out('<TABHERE>')

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
import templateCPP.functions.src.main_cpp as main

]]]
[[[end]]]
 *    Copyright (C)
[[[cog
A()
import datetime
cog.out(' ' + str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
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


/** \mainpage RoboComp::
[[[cog
A()
cog.out(component.name)
]]]
[[[end]]]
 *
 * \section intro_sec Introduction
 *
 * The
[[[cog
A()
cog.out(' ' + component.name)
Z()
]]]
[[[end]]]
 component...
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
 * cd
[[[cog
A()
cog.out(' ' + component.name)
Z()
]]]
[[[end]]]

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
 * Just: "${PATH_TO_BINARY}/
[[[cog
A()
cog.out(component.name)
Z()
]]]
[[[end]]]
 --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

[[[cog
cog.out(main.interface_includes(component.implements, 'I', True))
cog.out(main.interface_includes(component.subscribesTo, 'I', True))
cog.outl('')
cog.out(main.interface_includes(component.recursiveImports))
]]]
[[[end]]]


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class
[[[cog
A()
cog.out(' ' + component.name + ' ')
Z()
]]]
[[[end]]]
: public RoboComp::Application
{
public:
	[[[cog
	cog.out(component.name + ' (QString prfx) { prefix = prfx.toStdString(); }')
	]]]
	[[[end]]]
private:
	void initialize();
	std::string prefix;
	[[[cog
		if component.language.lower() == "cpp":
			cog.outl('MapPrx mprx;')
		else:
			cog.outl('TuplePrx tprx;')
	]]]
	[[[end]]]

public:
	virtual int run(int, char*[]);
};

void
[[[cog
A()
cog.out(' ::' + component.name)
Z()
]]]
[[[end]]]
::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int
[[[cog
A()
cog.out(' ::' + component.name)
Z()
]]]
[[[end]]]
::run(int argc, char* argv[])
{
[[[cog
	if component.gui is not None:
		cog.outl("#ifdef USE_QTGUI")
		cog.outl("<TABHERE>QApplication a(argc, argv);  // GUI application")
		cog.outl("#else")
		cog.outl("<TABHERE>QCoreApplication a(argc, argv);  // NON-GUI application")
		cog.outl("#endif")
	else:
		cog.outl("<TABHERE>QCoreApplication a(argc, argv);  // NON-GUI application")
]]]
[[[end]]]


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	[[[cog
	cog.out(main.proxy_ptr(component.publishes, component.language, 'pub'))
	cog.out(main.proxy_ptr(component.requires, component.language))
	]]]
	[[[end]]]

	string proxy, tmp;
	initialize();
	[[[cog
	proxy_list = []
	require_str, req_proxy_list = main.requires(component)
	proxy_list.extend(req_proxy_list)
	cog.out(require_str)
	cog.out(main.topic_manager_creation(component))
	]]]
    [[[end]]]
	[[[cog
	publish_str, pub_proxy_list = main.publish(component)
	cog.out(publish_str)
	proxy_list.extend(pub_proxy_list)

	if component.usingROS == True:
		cog.outl("ros::init(argc, argv, \""+component.name+"\");")

	cog.out(main.specificworker_creation(component.language, proxy_list))
	]]]
	[[[end]]]
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
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			[[[cog
				if component.language.lower() == "cpp":
					cog.outl("CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor);")
				else:
					cog.outl("auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);")
			]]]
			[[[end]]]
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}


		[[[cog
		cog.out(main.implements(component))
		cog.out(main.subscribes_to(component))
		]]]
		[[[end]]]

		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();
		[[[cog
		cog.out(main.unsubscribe_code(component))
		]]]
		[[[end]]]

		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "etc/config";
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
	[[[cog
	cog.outl('::' + component.name + ' app(prefix);')
	]]]
	[[[end]]]

	return app.main(argc, argv, configFile.c_str());
}
