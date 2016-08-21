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


/** \mainpage RoboComp::client4
 *
 * \section intro_sec Introduction
 *
 * The client4 component...
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
 * cd client4
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
 * Just: "${PATH_TO_BINARY}/client4 --Ice.Config=${PATH_TO_CONFIG_FILE}"
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
#include <qlog/qlog.h>
#include <fstream>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <asrI.h>

#include <RCMaster.h>
#include <Test.h>
#include <ASR.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

using namespace RoboCompRCMaster;
using namespace RoboCompTest;
using namespace RoboCompASR;



class client4 : public RoboComp::Application
{
public:
	client4 (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;
	Mapiface ifaces;

public:
	virtual int run(int, char*[]);
};

void ::client4::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::client4::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  // NON-GUI application


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);



	int status=EXIT_SUCCESS;

	testPrx test1_proxy;
	testPrx test2_proxy;
	rcmasterPrx rcmaster_proxy;

	string proxy, tmp,ComponentName;
	initialize();

	
	if (not GenericMonitor::configGetString(communicator(), prefix, "Ice.ProgramName", ComponentName, ""))
	{
		cout << "[" << PROGRAM_NAME << "]: Can't read Component Name\n";
		return EXIT_FAILURE;
	}

	// Remote object connection for rcmaster
	ifaces["rcmaster"] = ifaceData("rcmaster","rcmaster","rcmaster");
	try
	{
		string proxyStr = "rcmaster:tcp -h ";
		try
		{
			std::string port, host, line;
			ifstream infile(string(getenv("HOME"))+string("/.config/RoboComp/rcmaster.config"));
			getline(infile, line);
			std::size_t pos = line.find(":");
			host = line.substr (0,pos);
			proxyStr += line.substr (0,pos);
			proxyStr += " -p " + line.substr (pos+1);
			infile.close();
		}
		catch(...)
		{
			cout << "[" << PROGRAM_NAME << "]: Exception: " << "Cant get rcmaster proxy";
			return EXIT_FAILURE;
		}
		cout<<proxyStr<<endl;
		try
		{
			rcmaster_proxy = rcmasterPrx::uncheckedCast( communicator()->stringToProxy(proxyStr) );	
		}
		catch(Ice::SocketException)
		{
			cout << "[" << PROGRAM_NAME << "]: Exception: " << "RCmaster not running";
			return EXIT_FAILURE;
		}

	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("rcmasterProxy initialized Ok!");
	mprx["rcmasterProxy"] = (::IceProxy::Ice::Object*)(&rcmaster_proxy);

		// Remote conneciton for test1
	ifaces["test1"] = ifaceData("test1","test","client31");
	while (true)
	{
		try
		{
			interfaceList interfaces = rcmaster_proxy->getComp(ifaces["test1"].comp,"localhost");
			interfaceData iface;
			for(auto const &ifc : interfaces)
			{
				if (ifc.name == ifaces["test1"].name)
				{
					iface = ifc;
					break;
				}
			}
			string port = std::to_string(iface.port);
			string proxy = iface.name+":"+iface.protocol+" -h localhost "+" -p "+port;
			cout<<proxy<<endl;
			test1_proxy = testPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
			break;
		}
		catch (ComponentNotFound)
		{
			cout << "[" << PROGRAM_NAME << "]:" << "waiting for test1 interface"<<endl;
			sleep(3);
			continue;
		}
		catch(const Ice::Exception& ex)
		{
			cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
			return EXIT_FAILURE;
		}
	}
	rInfo("testProxy1 initialized Ok!");
	mprx["testProxy1"] = (::IceProxy::Ice::Object*)(&test1_proxy);//Remote server proxy creation example


	//Remote Conneciton for test2
	ifaces["test2"] = ifaceData("test2","test","client32");
	while (true)
	{
		try
		{
			interfaceList interfaces = rcmaster_proxy->getComp(ifaces["test2"].comp,"localhost");
			interfaceData iface;
			for(auto const &ifc : interfaces)
			{
				if (ifc.name == ifaces["test1"].name)
				{
					iface = ifc;
					break;
				}
			}
			string port = std::to_string(iface.port);
			string proxy = iface.name+":"+iface.protocol+" -h localhost "+" -p "+port;
			test2_proxy = testPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
			break;
		}
		catch (ComponentNotFound)
		{
			cout << "[" << PROGRAM_NAME << "]:" << "waiting for test interface"<<endl;
			sleep(3);
			continue;
		}
		catch(const Ice::Exception& ex)
		{
			cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
			return EXIT_FAILURE;
		}
	}
	rInfo("testProxy2 initialized Ok!");
	mprx["testProxy2"] = (::IceProxy::Ice::Object*)(&test2_proxy);//Remote server proxy creation example





	SpecificWorker *worker = new SpecificWorker(mprx, ifaces);
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
		// // Server adapter creation and publication
		// if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, ""))
		// {
		// 	cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
		// }
		// Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
		// CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		// adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		// adapterCommonBehavior->activate();

		// Register Compoennt 
        compData compInfo;
        compInfo.name = ComponentName;
        interfaceData idatap;
        idatap.name = "ASR";
        compInfo.interfaces.push_back(idatap);
        interfaceList idatas;
        rcmaster_proxy->registerComp(compInfo,false,true,idatas);
        map<string,string> portMap;
		for (auto const &idata :idatas)
        	portMap[idata.name] = std::to_string(idata.port);


		// Server adapter creation and publication
		//Activate ASR interface
		Ice::ObjectAdapterPtr adapterASR = communicator()->createObjectAdapterWithEndpoints("ASR","default -p "+portMap["ASR"]);
		ASRI *asr = new ASRI(worker);
		adapterASR->add(asr, communicator()->stringToIdentity("asr"));
		adapterASR->activate();
		cout << "[" << PROGRAM_NAME << "]: ASR adapter created in port " << tmp << endl;





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
	::client4 app(prefix);

	return app.main(argc, argv, configFile.c_str());
}

