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

from parseCDSL import *
includeDirectories = theIDSLPaths.split('#')
component = CDSLParsing.fromFile(theCDSL, includeDirectories=includeDirectories)


REQUIRE_STR = """
<TABHERE>try
<TABHERE>{
<TABHERE><TABHERE>if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL><PROXYNUMBER>Proxy", proxy, ""))
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>Proxy\\n";
<TABHERE><TABHERE>}
<TABHERE><TABHERE><C++_VERSION>
<TABHERE>}
<TABHERE>catch(const Ice::Exception& ex)
<TABHERE>{
<TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Exception creating proxy <NORMAL><PROXYNUMBER>: " << ex;
<TABHERE><TABHERE>return EXIT_FAILURE;
<TABHERE>}
<TABHERE>rInfo("<NORMAL>Proxy<PROXYNUMBER> initialized Ok!");
"""

SUBSCRIBESTO_STR = """
<TABHERE><TABHERE>// Server adapter creation and publication
<TABHERE><TABHERE><CHANGE1> <LOWER>_topic;
<TABHERE><TABHERE><CHANGE2> <PROXYNAME>;
<TABHERE><TABHERE>try
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE>if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL>Topic.Endpoints", tmp, ""))
<TABHERE><TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>Proxy";
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE>Ice::ObjectAdapterPtr <NORMAL>_adapter = communicator()->createObjectAdapterWithEndpoints("<LOWER>", tmp);
<TABHERE><TABHERE><TABHERE><NORMAL>Ptr <LOWER>I_ = <CHANGE3>(worker);
<TABHERE><TABHERE><TABHERE><CHANGE4> <PROXYNAME> = <NORMAL>_adapter->addWithUUID(<LOWER>I_)->ice_oneway();
<TABHERE><TABHERE><TABHERE>if(!<LOWER>_topic)
<TABHERE><TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><TABHERE>try {
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager->create("<NORMAL>");
<TABHERE><TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE><TABHERE>catch (const IceStorm::TopicExists&) {
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>//Another client created the topic
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>try{
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\\n";
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager->retrieve("<NORMAL>");
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>catch(const IceStorm::NoSuchTopic&)
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\\n";
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>//Error. Topic does not exist
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE><TABHERE>IceStorm::QoS qos;
<TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic->subscribeAndGetPublisher(qos, <PROXYNAME>);
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE><NORMAL>_adapter->activate();
<TABHERE><TABHERE>}
<TABHERE><TABHERE>catch(const IceStorm::NoSuchTopic&)
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Error creating <NORMAL> topic.\\n";
<TABHERE><TABHERE><TABHERE>//Error. Topic does not exist
<TABHERE><TABHERE>}
"""

PUBLISHES_STR = """
<TABHERE>while (!<LOWER>_topic)
<TABHERE>{
<TABHERE><TABHERE>try
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager->retrieve("<NORMAL>");
<TABHERE><TABHERE>}
<TABHERE><TABHERE>catch (const IceStorm::NoSuchTopic&)
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: ERROR retrieving <NORMAL> topic. \\n";
<TABHERE><TABHERE><TABHERE>try
<TABHERE><TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager->create("<NORMAL>");
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE>catch (const IceStorm::TopicExists&){
<TABHERE><TABHERE><TABHERE><TABHERE>// Another client created the topic.
<TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: ERROR publishing the <NORMAL> topic. It's possible that other component have created\\n";
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE>}
<TABHERE>}
"""

IMPLEMENTS_STR = """
<TABHERE><TABHERE>try
<TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE>// Server adapter creation and publication
<TABHERE><TABHERE><TABHERE>if (not GenericMonitor::configGetString(communicator(), prefix, "<NORMAL>.Endpoints", tmp, ""))
<TABHERE><TABHERE><TABHERE>{
<TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy <NORMAL>";
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE>Ice::ObjectAdapterPtr adapter<NORMAL> = communicator()->createObjectAdapterWithEndpoints("<NORMAL>", tmp);
<TABHERE><TABHERE><TABHERE><C++_VERSION>
<TABHERE><TABHERE><TABHERE>adapter<NORMAL>->add(<LOWER>, Ice::stringToIdentity("<LOWER>"));
<TABHERE><TABHERE><TABHERE>adapter<NORMAL>->activate();
<TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: <NORMAL> adapter created in port " << tmp << endl;
<TABHERE><TABHERE><TABHERE>}
<TABHERE><TABHERE><TABHERE>catch (const IceStorm::TopicExists&){
<TABHERE><TABHERE><TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for <NORMAL>\\n";
<TABHERE><TABHERE><TABHERE>}
"""

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
cog.out(component['name'])
]]]
[[[end]]]
 *
 * \section intro_sec Introduction
 *
 * The
[[[cog
A()
cog.out(' ' + component['name'])
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
cog.out(' ' + component['name'])
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
cog.out(component['name'])
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
for ima in component['implements']:
	if type(ima) == str:
		im = ima
	else:
		im = ima[0]
	if communicationIsIce(ima):
		cog.outl('#include <'+im.lower()+'I.h>')

for subscribe in component['subscribesTo']:
	subs = subscribe
	while type(subs) != type(''):
		subs = subs[0]
	if communicationIsIce(subscribe):
		cog.outl('#include <'+subs.lower()+'I.h>')

cog.outl('')

for imp in component['recursiveImports']:
	incl = imp.split('/')[-1].split('.')[0]
	cog.outl('#include <'+incl+'.h>')

]]]
[[[end]]]


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class
[[[cog
A()
cog.out(' ' + component['name'] + ' ')
Z()
]]]
[[[end]]]
: public RoboComp::Application
{
public:
[[[cog
cog.out('<TABHERE>' + component['name'] + ' (QString prfx) { prefix = prfx.toStdString(); }')
]]]
[[[end]]]
private:
	void initialize();
	std::string prefix;
[[[cog
	if component['language'].lower() == "cpp":
		cog.outl('<TABHERE>MapPrx mprx;')
	else:
		cog.outl('<TABHERE>TuplePrx tprx;')
]]]
[[[end]]]

public:
	virtual int run(int, char*[]);
};

void
[[[cog
A()
cog.out(' ::' + component['name'])
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
cog.out(' ::' + component['name'])
Z()
]]]
[[[end]]]
::run(int argc, char* argv[])
{
[[[cog
	if component['gui'] != "none":
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
for namea, num in getNameNumber(component['requires'] + component['publishes']):
	if type(namea) == str:
		name = namea
	else:
		name = namea[0]
		if communicationIsIce(namea):
			if component['language'].lower() == "cpp":
				cog.outl('<TABHERE>'+name+'Prx '+name.lower()+num +'_proxy;')
			else:
				cog.outl('<TABHERE>'+name+'PrxPtr '+name.lower()+num +'_proxy;')
try:
	if isAGM1Agent(component):
		if component['language'].lower() == "cpp":
			cog.outl("<TABHERE>AGMExecutivePrx agmexecutive_proxy;")
		else:
			cog.outl("<TABHERE>AGMExecutivePrxPtr agmexecutive_proxy;")
except:
	pass
]]]
[[[end]]]

	string proxy, tmp;
	initialize();

[[[cog
proxy_list = []
for namea, num in getNameNumber(component['requires']):
	if type(namea) == str:
		name = namea
	else:
		name = namea[0]
	if communicationIsIce(namea):
		if component['language'].lower() == "cpp":
			cpp = "<PROXYNAME>_proxy = <NORMAL>Prx::uncheckedCast( communicator()->stringToProxy( proxy ) );"
		else:
			cpp = "<PROXYNAME>_proxy = Ice::uncheckedCast<<NORMAL>Prx>( communicator()->stringToProxy( proxy ) );"
	
		w = REQUIRE_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", name).replace("<LOWER>", name.lower()).replace("<PROXYNAME>", name.lower()+num).replace("<PROXYNUMBER>", num)
		cog.outl(w)
	if component['language'].lower() == "cpp":
		cog.outl("<TABHERE>mprx[\""+name+"Proxy"+num+"\"] = (::IceProxy::Ice::Object*)(&"+name.lower()+num+"_proxy);//Remote server proxy creation example");
	else:
		proxy_list.append(name.lower()+"_proxy")
	
need_topic=False
for pub in component['publishes']:
	if communicationIsIce(pub):
		need_topic = True
for pub in component['subscribesTo']:
	if communicationIsIce(pub):
		need_topic = True
if need_topic:
	if component['language'].lower() == "cpp":
		cog.outl('<TABHERE>IceStorm::TopicManagerPrx topicManager;')
	else:
		cog.outl('<TABHERE>IceStorm::TopicManagerPrxPtr topicManager;')
	cog.outl('<TABHERE>try')
	cog.outl('<TABHERE>{')
	if component['language'].lower() == "cpp":
		cog.outl('<TABHERE><TABHERE>topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));')
	else:
		cog.outl('<TABHERE><TABHERE>topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));')
	cog.outl('<TABHERE>}')
	cog.outl('<TABHERE>catch (const Ice::Exception &ex)')
	cog.outl('<TABHERE>{')
	cog.outl('<TABHERE><TABHERE>cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;')
	cog.outl('<TABHERE><TABHERE>return EXIT_FAILURE;')
	cog.outl('<TABHERE>}')


for pba in component['publishes']:
	if type(pba) == str:
		pb = pba
	else:
		pb = pba[0]
	if communicationIsIce(pba):
		if component['language'].lower() == "cpp":
			cog.outl("<TABHERE>IceStorm::TopicPrx "+pb.lower() + "_topic;")
		else:
			cog.outl("<TABHERE>std::shared_ptr<IceStorm::TopicPrx> "+pb.lower() + "_topic;")
		w = PUBLISHES_STR.replace("<NORMAL>", pb).replace("<LOWER>", pb.lower())
		cog.outl(w)
		if component['language'].lower() == "cpp":
			cog.outl("<TABHERE>Ice::ObjectPrx " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();")
			cog.outl("<TABHERE>" + pb.lower() + "_proxy = " + pb + "Prx::uncheckedCast(" + pb.lower() + "_pub);")
			cog.outl("<TABHERE>mprx[\"" + pb + "Pub\"] = (::IceProxy::Ice::Object*)(&" + pb.lower() + "_proxy);")
		else:
			cog.outl("<TABHERE>auto " + pb.lower() + "_pub = " + pb.lower() + "_topic->getPublisher()->ice_oneway();")
			cog.outl("<TABHERE>" + pb.lower() + "_proxy = Ice::uncheckedCast<" + pb + "Prx>(" + pb.lower() + "_pub);")
			proxy_list.append(pb.lower() + "_proxy")


if component['usingROS'] == True:
	cog.outl("<TABHERE>ros::init(argc, argv, \""+component['name']+"\");")


]]]
[[[end]]]

[[[cog
    if component['language'].lower() == "cpp":
        cog.outl("<TABHERE>SpecificWorker *worker = new SpecificWorker(mprx);")
    else:
        if proxy_list:
            cog.outl("<TABHERE>tprx = std::make_tuple(" + ",".join(proxy_list) + ");")
        else:
            cog.outl("<TABHERE>tprx = NULL;")
        cog.outl("<TABHERE>SpecificWorker *worker = new SpecificWorker(tprx);")
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
				if component['language'].lower() == "cpp":
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
for ima in component['implements']:
	if type(ima) == str:
		im = ima
	else:
		im = ima[0]
	if communicationIsIce(ima):
		if component['language'].lower() == "cpp":
			cpp = "<NORMAL>I *<LOWER> = new <NORMAL>I(worker);"
		else:
			cpp = "auto <LOWER> = std::make_shared<<NORMAL>I>(worker);"
		w = IMPLEMENTS_STR.replace("<C++_VERSION>", cpp).replace("<NORMAL>", im).replace("<LOWER>", im.lower())
		cog.outl(w)
]]]
[[[end]]]

[[[cog
for name, num in getNameNumber(component['subscribesTo']):
	nname = name
	while type(nname) != type(''):
		nname = name[0]
	if communicationIsIce(name):
		if component['language'].lower() == "cpp":
			change1 = "IceStorm::TopicPrx"
			change2 = "Ice::ObjectPrx"
			change3 = " new <NORMAL>I"
			change4 = "Ice::ObjectPrx"
		else:
			change1 = "std::shared_ptr<IceStorm::TopicPrx>"
			change2 = "Ice::ObjectPrxPtr"
			change3 = " std::make_shared <<NORMAL>I>"
			change4 = "auto"
		
		w = SUBSCRIBESTO_STR.replace("<CHANGE1>", change1).replace("<CHANGE2>", change2).replace("<CHANGE3>", change3).replace("<CHANGE4>", change4).replace("<NORMAL>", nname).replace("<LOWER>", nname.lower()).replace("<PROXYNAME>", nname.lower()+num).replace("<PROXYNUMBER>", num)
		cog.out(w)
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
for sub in component['subscribesTo']:
	nname = sub
	while type(nname) != type(''):
		nname = sub[0]
	if communicationIsIce(sub):
		cog.outl("<TABHERE><TABHERE>try")
		cog.outl("<TABHERE><TABHERE>{")
		cog.outl("<TABHERE><TABHERE><TABHERE>std::cout << \"Unsubscribing topic: "+nname.lower()+" \" <<std::endl;")
		cog.outl("<TABHERE><TABHERE><TABHERE>"+ nname.lower() + "_topic->unsubscribe( "+ nname.lower() +" );" )
		cog.outl("<TABHERE><TABHERE>}")
		cog.outl("<TABHERE><TABHERE>catch(const Ice::Exception& ex)")
		cog.outl("<TABHERE><TABHERE>{")
		cog.outl("<TABHERE><TABHERE><TABHERE>std::cout << \"ERROR Unsubscribing topic: "+nname.lower()+" \" <<std::endl;")
		cog.outl("<TABHERE><TABHERE>}")

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

[[[cog
A()
cog.out('<TABHERE>::' + component['name'] + ' ')
Z()
]]]
[[[end]]]
app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
