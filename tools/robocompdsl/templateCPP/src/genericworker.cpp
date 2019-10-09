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

from parseSMDSL import *
from parseCDSL import *
includeDirectories = theIDSLPaths.split('#')
component = CDSLParsing.fromFile(theCDSL, includeDirectories=includeDirectories)
sm = SMDSLparsing.fromFile(component['statemachine'])
if sm is None:
    component['statemachine'] = None
if component is None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
pool = IDSLPool(theIDSLs, includeDirectories)


]]]
[[[end]]]
 *    Copyright (C)
[[[cog
A()
import datetime
cog.out(str(datetime.date.today().year))
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
[[[cog
if component['language'].lower() == 'cpp':
	cog.outl("GenericWorker::GenericWorker(MapPrx& mprx) :")
else:
	cog.outl("GenericWorker::GenericWorker(TuplePrx tprx) :")
if component['gui'] is not None:
	cog.outl("""#ifdef USE_QTGUI
Ui_guiDlg()
#else
QObject()
#endif
""")
else:
	cog.outl("QObject()")
]]]
[[[end]]]
{

[[[cog
if sm is not None:
    codaddTransition = ""
    codaddState = ""
    codConnect = ""
    codsetInitialState = ""
    states = ""
    if sm['machine']['contents']['transitions'] is not None:
        for transi in sm['machine']['contents']['transitions']:
            for dest in transi['dests']:
                codaddTransition += "<TABHERE>" + transi['src'] + "State->addTransition(" + "this, SIGNAL(t_"+transi['src'] + "_to_" + dest+"()), " + dest + "State);\n"
    if sm['substates'] is not None:
        for substates in sm['substates']:
            if substates['contents']['transitions'] is not None:
                for transi in substates['contents']['transitions']:
                    for dest in transi['dests']:
                        codaddTransition += "<TABHERE>" + transi['src'] + "State->addTransition(" + "this, SIGNAL(t_"+transi['src'] + "_to_" + dest+"()), " + dest + "State);\n"
    if sm['machine']['contents']['states'] is not None:
        for state in sm['machine']['contents']['states']:
            codaddState += "<TABHERE>" + sm['machine']['name'] +  ".addState(" + state + "State);\n"
            codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
            states += state + ","
    if sm['machine']['contents']['initialstate'] is not None:
        state = sm['machine']['contents']['initialstate']
        codaddState += "<TABHERE>" + sm['machine']['name'] +  ".addState(" + state + "State);\n"
        codsetInitialState += "<TABHERE>" + sm['machine']['name'] +  ".setInitialState(" + state +"State);\n"
        codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
        states += state + ","
    if sm['machine']['contents']['finalstate'] is not None:
        state = sm['machine']['contents']['finalstate']
        codaddState += "<TABHERE>" + sm['machine']['name'] +  ".addState(" + state + "State);\n"
        codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
        states += state + ","

    if sm['substates'] is not None:
        for substates in sm['substates']:
            if substates['contents']['initialstate'] is not None:
                state = substates['contents']['initialstate']
                codsetInitialState += "<TABHERE>" + substates['parent'] +  "State->setInitialState(" + state +"State);\n"
                codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                states += state + ","
            if substates['contents']['finalstate'] is not None:
                state = substates['contents']['finalstate']
                codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                states += state + ","
            if substates['contents']['states'] is not None:
                for state in substates['contents']['states']:
                    codConnect += "<TABHERE>QObject::connect(" + state + "State, SIGNAL(entered()), this, SLOT(sm_" + state + "()));\n"
                    states += state + ","
    if sm['machine']['default']:
        codConnect += "<TABHERE>QObject::connect(&timer, SIGNAL(timeout()), this, SIGNAL(t_compute_to_compute()));\n"
    cog.outl("//Initialization State machine")
    cog.outl(codaddTransition)
    cog.outl(codaddState)
    cog.outl(codsetInitialState)
    cog.outl(codConnect)
    cog.outl("//------------------")	
]]]
[[[end]]]
[[[cog
cont = 0
for name, num in getNameNumber(component['requires']):
	if communicationIsIce(name):
		if component['language'].lower() == 'cpp':
			cog.outl("<TABHERE>"+name.lower()+num+"_proxy = (*("+name+"Prx*)mprx[\""+name+"Proxy"+num+"\"]);")
		else:
			cog.outl("<TABHERE>"+name.lower()+num+"_proxy = std::get<" + str(cont) + ">(tprx);")
	cont = cont + 1

for name, num in getNameNumber(component['publishes']):
	if communicationIsIce(name):
		if component['language'].lower() == 'cpp':
			cog.outl("<TABHERE>"+name.lower()+num+"_pubproxy = (*("+name+"Prx*)mprx[\""+name+"Pub"+num+"\"]);")
		else:
			cog.outl("<TABHERE>"+name.lower()+num+"_pubproxy = std::get<" + str(cont) + ">(tprx);")
	cont = cont + 1
]]]
[[[end]]]

	mutex = new QMutex(QMutex::Recursive);

[[[cog
if component['usingROS'] == True:
	#INICIALIZANDO SUBSCRIBERS
	for imp in component['subscribesTo']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						s = "\""+mname+"\""
						if nname in component['iceInterfaces']:
							cog.outl("<TABHERE>"+nname+"_"+mname+" = node.subscribe("+s+", 1000, &GenericWorker::ROS"+mname+", this);")
						else:
							cog.outl("<TABHERE>"+nname+"_"+mname+" = node.subscribe("+s+", 1000, &GenericWorker::"+mname+", this);")
	#INICIALIZANDO IMPLEMENTS
	for imp in component['implements']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						s = "\""+mname+"\""
						if nname in component['iceInterfaces']:
							cog.outl("<TABHERE>"+nname+"_"+mname+" = node.advertiseService("+s+", &GenericWorker::ROS"+mname+", this);")
						else:
							cog.outl("<TABHERE>"+nname+"_"+mname+" = node.advertiseService("+s+", &GenericWorker::"+mname+", this);")
if 'publishes' in component:
	for publish in component['publishes']:
		pubs = publish
		while type(pubs) != type(''):
			pubs = pubs[0]
		if not communicationIsIce(publish):
			if pubs in component['iceInterfaces']:
				cog.outl("<TABHERE>"+pubs.lower()+"_rosproxy = new Publisher"+pubs+"(&node);")
			else:
				cog.outl("<TABHERE>"+pubs.lower()+"_proxy = new Publisher"+pubs+"(&node);")
if 'requires' in component:
	for require in component['requires']:
		req = require
		while type(req) != type(''):
			req = req[0]
		if not communicationIsIce(require):
			if req in component['iceInterfaces']:
				cog.outl("<TABHERE>"+req.lower()+"_rosproxy = new ServiceClient"+req+"(&node);")
			else:
				cog.outl("<TABHERE>"+req.lower()+"_proxy = new ServiceClient"+req+"(&node);")
if component['gui'] is not None:
	cog.outl("""<TABHERE>#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif""")
]]]
[[[end]]]
	Period = BASIC_PERIOD;
[[[cog
if (sm is not None and sm['machine']['default'] is True) or component['statemachine'] is None:
	cog.outl("<TABHERE>connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));")
]]]
[[[end]]]

}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}

[[[cog
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("""RoboCompPlanning::Action GenericWorker::createAction(std::string s)
{
	// Remove useless characters
	char chars[]="()";
		for (unsigned int i=0; i<strlen(chars); ++i)
	{
		s.erase(std::remove(s.begin(), s.end(), chars[i]), s.end());
	}

		// Initialize string parsing
	RoboCompPlanning::Action ret;
	istringstream iss(s);

	// Get action (first segment)
	if (not iss)
	{
		printf("agent %s: received invalid action (%s) -> (%d)\\n", PROGRAM_NAME, __FILE__, __LINE__);
		exit(-1);
	}
	else
	{
		iss >> ret.name;
	}

	do
	{
		std::string ss;
		iss >> ss;
		ret.symbols.push_back(ss);
	} while (iss);

	return ret;
}


bool GenericWorker::activate(const BehaviorParameters &prs)
{
	printf("Worker::activate\\n");
	mutex->lock();
	p = prs;
	active = true;
	iter = 0;
	mutex->unlock();
	return active;
}

bool GenericWorker::deactivate()
{
	printf("Worker::deactivate\\n");
	mutex->lock();
	active = false;
	iter = 0;
	mutex->unlock();
	return active;
}

bool GenericWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		// Action
		p.action = createAction(params["action"].value);

		// Fill received plan
		p.plan.clear();
		QStringList actionList = QString::fromStdString(params["plan"].value).split(QRegExp("[()]+"), QString::SkipEmptyParts);
		for (int32_t actionString=0; actionString<actionList.size(); actionString++)
		{
			std::vector<string> elementsVec;
			QStringList elements = actionList[actionString].remove(QChar('\\n')).split(QRegExp("\\\\s+"), QString::SkipEmptyParts);
			for (int32_t elem=0; elem<elements.size(); elem++)
			{
				elementsVec.push_back(elements[elem].toStdString());
			}
			p.plan.push_back(elementsVec);
		}
	}
	catch (...)
	{
		return false;
	}

	// Check if we should reactivate the component
	if (isActive())
	{
		activate(p);
		reactivated = true;
	}

	return true;
}""")
except:
	pass

]]]
[[[end]]]
