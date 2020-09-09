/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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


/** \mainpage RoboComp::testcomp
 *
 * \section intro_sec Introduction
 *
 * The testcomp component...
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
 * cd testcomp
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
 * Just: "${PATH_TO_BINARY}/testcomp --Ice.Config=${PATH_TO_CONFIG_FILE}"
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

#include <handdetectionI.h>
#include <apriltagsI.h>

#include <GenericBase.h>
#include <JointMotor.h>



class testcomp : public RoboComp::Application
{
public:
	testcomp (QString prfx, bool startup_check) { prefix = prfx.toStdString(); this->startup_check_flag=startup_check; }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::testcomp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::testcomp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


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

	RoboCompAprilBasedLocalization::AprilBasedLocalizationPrx aprilbasedlocalization_pubproxy;
	RoboCompCameraSimple::CameraSimplePrx camerasimple_proxy;
	RoboCompRGBD::RGBDPrx rgbd_proxy;

	string proxy, tmp;
	initialize();

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "CameraSimpleProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraSimpleProxy\n";
		}
		camerasimple_proxy = RoboCompCameraSimple::CameraSimplePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy CameraSimple: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("CameraSimpleProxy initialized Ok!");

	mprx["CameraSimpleProxy"] = (::IceProxy::Ice::Object*)(&camerasimple_proxy);//Remote server proxy creation example

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "RGBDProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy RGBDProxy\n";
		}
		rgbd_proxy = RoboCompRGBD::RGBDPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy RGBD: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("RGBDProxy initialized Ok!");

	mprx["RGBDProxy"] = (::IceProxy::Ice::Object*)(&rgbd_proxy);//Remote server proxy creation example

	IceStorm::TopicManagerPrx topicManager;
	try
	{
		topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << endl;
		return EXIT_FAILURE;
	}
	IceStorm::TopicPrx aprilbasedlocalization_topic;

	while (!aprilbasedlocalization_topic)
	{
		try
		{
			aprilbasedlocalization_topic = topicManager->retrieve("AprilBasedLocalization");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR retrieving AprilBasedLocalization topic. \n";
			try
			{
				aprilbasedlocalization_topic = topicManager->create("AprilBasedLocalization");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
				cout << "[" << PROGRAM_NAME << "]: ERROR publishing the AprilBasedLocalization topic. It's possible that other component have created\n";
			}
		}
		catch(const IceUtil::NullHandleException&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
			"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
			return EXIT_FAILURE;
		}
	}

	Ice::ObjectPrx aprilbasedlocalization_pub = aprilbasedlocalization_topic->getPublisher()->ice_oneway();
	aprilbasedlocalization_pubproxy = RoboCompAprilBasedLocalization::AprilBasedLocalizationPrx::uncheckedCast(aprilbasedlocalization_pub);
	mprx["AprilBasedLocalizationPub"] = (::IceProxy::Ice::Object*)(&aprilbasedlocalization_pubproxy);

	SpecificWorker *worker = new SpecificWorker(mprx, startup_check_flag);
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
			CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "HandDetection.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy HandDetection";
			}
			Ice::ObjectAdapterPtr adapterHandDetection = communicator()->createObjectAdapterWithEndpoints("HandDetection", tmp);
			HandDetectionI *handdetection = new HandDetectionI(worker);
			adapterHandDetection->add(handdetection, Ice::stringToIdentity("handdetection"));
			adapterHandDetection->activate();
			cout << "[" << PROGRAM_NAME << "]: HandDetection adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for HandDetection\n";
		}


		// Server adapter creation and publication
		IceStorm::TopicPrx apriltags_topic;
		Ice::ObjectPrx apriltags;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "AprilTagsTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilTagsProxy";
			}
			Ice::ObjectAdapterPtr AprilTags_adapter = communicator()->createObjectAdapterWithEndpoints("apriltags", tmp);
			RoboCompAprilTags::AprilTagsPtr apriltagsI_ =  new AprilTagsI(worker);
			Ice::ObjectPrx apriltags = AprilTags_adapter->addWithUUID(apriltagsI_)->ice_oneway();
			if(!apriltags_topic)
			{
				try {
					apriltags_topic = topicManager->create("AprilTags");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						apriltags_topic = topicManager->retrieve("AprilTags");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				catch(const IceUtil::NullHandleException&)
				{
					cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
					"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
					return EXIT_FAILURE;
				}
				IceStorm::QoS qos;
				apriltags_topic->subscribeAndGetPublisher(qos, apriltags);
			}
			AprilTags_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating AprilTags topic.\n";
			//Error. Topic does not exist
		}


		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: apriltags " <<std::endl;
			apriltags_topic->unsubscribe( apriltags );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: apriltags " << ex.what()<<std::endl;
		}


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
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{
	    QString initIC = QString("--Ice.Config=");
	    for (int i = 1; i < argc; ++i)
		{
		    arg = argv[i];
            if (arg.find(initIC.toStdString(), 0) == 0)
            {
                configFile = QString::fromStdString(arg).remove(0, initIC.size());
            }
        }

        // Search in argument list for --prefix= argument (if exist)
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

        // Search in argument list for --test argument (if exist)
        QString startup = QString("--startup-check");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) == 0)
			{
				startup_check_flag = true;
				cout << "Startup check = True"<< endl;
			}
		}

	}
	::testcomp app(prefix, startup_check_flag);

	return app.main(argc, argv, configFile.toLocal8Bit().data());
}
