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
from parseIDSL import *
component = CDSLParsing.fromFile(theCDSL)
sm = SMDSLparsing.fromFile(component['statemachine'])
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)



pool = IDSLPool(theIDSLs)


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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

[[[cog
if component['gui'] != 'none':
	cog.outl("#include <ui_mainUI.h>")
]]]
[[[end]]]
[[[cog
if component['statemachine'] != 'none':
	cog.outl("#include <qt4/QtCore/qstatemachine.h>\n#include <qt4/QtCore/qstate.h>")
]]]
[[[end]]]
#include <CommonBehavior.h>
[[[cog

for m in pool.modulePool:
	cog.outl("#include <"+m+".h>")

]]]
[[[end]]]

[[[cog

try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("#include <agm.h>")
		cog.outl("#include <agmInner/agmInner.h>")


except:
	pass

]]]
[[[end]]]


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

[[[cog

pool = IDSLPool(theIDSLs)
for m in pool.modulePool:
	cog.outl("using namespace "+pool.modulePool[m]['name']+";")

]]]
[[[end]]]

[[[cog

try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("""
		struct BehaviorParameters 
		{
			RoboCompPlanning::Action action;
			std::vector< std::vector <std::string> > plan;
		};""")
except:
	pass

]]]
[[[end]]]



class GenericWorker : 
[[[cog
if component['gui'] != 'none':
	cog.outl("""#ifdef USE_QTGUI
public QWidget, public Ui_guiDlg
#else
public QObject
#endif""")
else:
	cog.outl("public QObject")
]]]
[[[end]]]
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
[[[cog

try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>bool activate(const BehaviorParameters& parameters);")
		cog.outl("<TABHERE>bool deactivate();")
		cog.outl("<TABHERE>bool isActive() { return active; }")
		cog.outl("<TABHERE>RoboCompAGMWorldModel::BehaviorResultType status();")
except:
	pass

]]]
[[[end]]]
	

[[[cog
for name, num in getNameNumber(component['requires']+component['publishes']):
	cog.outl('<TABHERE>'+name+'Prx '+name.lower()+num +'_proxy;')
]]]
[[[end]]]

[[[cog
if 'implements' in component:
	for imp in component['implements']:
		module = pool.moduleProviding(imp)
		for interface in module['interfaces']:
			if interface['name'] == imp:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					paramStrA = ''
					for p in method['params']:
						# delim
						if paramStrA == '': delim = ''
						else: delim = ', '
						# decorator
						ampersand = '&'
						if p['decorator'] == 'out':
							const = ''
						else:
							const = 'const '
							if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
								ampersand = ''
						# STR
						paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
					cog.outl("<TABHERE>virtual " + method['return'] + ' ' + method['name'] + '(' + paramStrA + ") = 0;")

if 'subscribesTo' in component:
	for imp in component['subscribesTo']:
		nname = imp
		while type(nname) != type(''):			
			nname = nname[0]
		if communicationIsIce(nname):
			module = pool.moduleProviding(nname)
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						paramStrA = ''
						for p in method['params']:
							# delim
							if paramStrA == '': delim = ''
							else: delim = ', '
							# decorator
							ampersand = '&'
							if p['decorator'] == 'out':
								const = ''
							else:
								const = 'const '
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
							cog.outl("<TABHERE>virtual " + method['return'] + ' ' + method['name'] + '(' + paramStrA + ") = 0;")
		else:
			cog.outl("<TABHERE>virtual ROS" + method['return'] + ' ' + method['name'] + "() = 0;")
	

				
]]]
[[[end]]]


protected:
[[[cog
if sm is not "none":
    codQState = ""
    codQStateParallel = ""
    codQStateMachine = ""
    codQFinalState = ""

    codQStateMachine = "<TABHERE>QStateMachine " + sm['machine']['name'] + ";\n"

    for state in sm['machine']['contents']['states']:
        aux = "<TABHERE>QState *" + state + " = new QState();\n"
        if sm['substates'] is not "none":
            for substates in sm['substates']:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        aux = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates);\n"
                        break
        if "ParallelStates" in aux:
            codQStateParallel += aux
        else:
            codQState += aux
    if sm['machine']['contents']['initialstate'] != "none":
        state = sm['machine']['contents']['initialstate'][0]
        aux = "<TABHERE>QState *" + state + " = new QState();\n"
        if sm['substates'] is not "none":
            for substates in sm['substates']:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        aux = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates);\n"
                        break
        if "ParallelStates" in aux:
            codQStateParallel += aux
        else:
            codQState += aux


    if sm['machine']['contents']['finalstate'] != "none":
        state = sm['machine']['contents']['finalstate'][0]
        codQFinalState +="<TABHERE>QFinalState *" + state + " = new QFinalState();\n"


    if sm['substates'] != "none":
        for substates in sm['substates']:
            for state in substates['contents']['states']:
                aux = "<TABHERE>QState *" + state + " = new QState(" + substates['parent'] +");\n"
                for sub in sm['substates']:
                    if state == sub['parent']:
                        if sub['parallel'] is "parallel":
                            aux = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates, " + substates['parent'] +");\n"
                            break
                if "ParallelStates" in aux:
                    codQStateParallel += aux
                else:
                    codQState += aux
            if substates['contents']['initialstate'] != "none":
                aux = "<TABHERE>QState *" + substates['contents']['initialstate'][0] + " = new QState(" + substates['parent'][0] + ");\n"
                for sub in sm['substates']:
                    if state == sub['parent']:
                        if sub['parallel'] is "parallel":
                            aux = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates, " + substates['parent'] + ");\n"
                            break
                if "ParallelStates" in aux:
                    codQStateParallel += aux
                else:
                    codQState += aux
            if substates['contents']['finalstate'] != "none":
                codQFinalState += "<TABHERE>QFinalState *" + substates['contents']['finalstate'][0] + " = new QFinalState(" +substates['parent'][0] + ");\n"

    cog.outl("//State Machine")
    cog.outl(codQStateMachine)
    cog.outl(codQState)
    cog.outl(codQFinalState)
    cog.outl(codQStateParallel)
    cog.outl("//-------------------------")


]]]
[[[end]]]

	QTimer timer;
	int Period;
[[[cog
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>bool active;")
		cog.outl("<TABHERE>AGMModel::SPtr worldModel;")
		cog.outl("<TABHERE>BehaviorParameters p;")
		cog.outl("<TABHERE>ParameterMap params;")
		cog.outl("<TABHERE>AgmInner agmInner;")
		cog.outl("<TABHERE>int iter;")
		cog.outl("<TABHERE>bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);")
		cog.outl("<TABHERE>RoboCompPlanning::Action createAction(std::string s);")
except:
	pass

]]]
[[[end]]]

public slots:
	virtual void compute() = 0;
[[[cog
if component['statemachine'] != 'none':
    codVirtuals = ""
    for state in sm['machine']['contents']['states']:
        codVirtuals += "<TABHERE>virtual void fun_" + state + "() = 0;\n"
    if sm['machine']['contents']['initialstate'] != "none":
        codVirtuals += "<TABHERE>virtual void fun_" + sm['machine']['contents']['initialstate'][0] + "() = 0;\n"
    if sm['machine']['contents']['finalstate'] != "none":
        codVirtuals += "<TABHERE>virtual void fun_" + sm['machine']['contents']['finalstate'][0] + "() = 0;\n"
    if sm['substates'] != "none":
        for substates in sm['substates']:
            for state in substates['contents']['states']:
                codVirtuals += "<TABHERE>virtual void fun_" + state + "() = 0;\n"
            if substates['contents']['initialstate'] != "none":
                codVirtuals += "<TABHERE>virtual void fun_" + substates['contents']['initialstate'][0] + "() = 0;\n"
            if substates['contents']['finalstate'] != "none":
                codVirtuals += "<TABHERE>virtual void fun_" + substates['contents']['finalstate'][0] + "() = 0;\n"
    cog.outl("//Slots funtion State Machine")
    cog.outl(codVirtuals)
    cog.outl("//-------------------------")
]]]
[[[end]]]
signals:
	void kill();
[[[cog
if component['statemachine'] != 'none':
    codsignals = ""
    if sm['machine']['contents']['transition'] != "none":
        for transi in sm['machine']['contents']['transition']:
            for dest in transi['dest']:
                codsignals += "<TABHERE>void " +  transi['src'] + "to" + dest + "();\n"
    if sm['substates']!="none":
        for substates in sm['substates']:
            if substates['contents']['transition'] != "none":
                for transi in substates['contents']['transition']:
                    for dest in transi['dest']:
                        codsignals += "<TABHERE>void " + transi['src'] + "to" + dest + "();\n"
    cog.outl("//Signals for State Machine")
    cog.outl(codsignals)
    cog.outl("//-------------------------")
]]]
[[[end]]]
};

#endif