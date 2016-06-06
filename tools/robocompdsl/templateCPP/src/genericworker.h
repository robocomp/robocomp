/*
[[[cog
from parseSMDSL import *
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
    cog.outl("<TABHERE>QStateMachine " + sm['machine']['name'] + ";")

    cod = "<TABHERE>QState"
    for state in sm['machine']['contents']['states']:
        cod = "<TABHERE>QState *" + state + " = new QState();"
        if sm['substates'] is not "none":
            for substates in sm['substates']:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        cod = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates);"
                        break
        cog.outl(cod)

    if sm['machine']['contents']['initialstate'] != "none":
        state = sm['machine']['contents']['initialstate'][0]
        cod = "<TABHERE>QState *" + state + " = new QState();"
        if sm['substates'] is not "none":
            for substates in sm['substates']:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        cod = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates);"
                        break
        cog.outl(cod)



    if sm['machine']['contents']['finalstate'] != "none":
        state = sm['machine']['contents']['finalstate'][0]
        cog.outl("<TABHERE>QFinalState *" + state + " = new QFinalState();")


    if sm['substates'] != "none":
        for substates in sm['substates']:
            for state in substates['contents']['states']:
                cod = "<TABHERE>QState *" + state + " = new QState(" + substates['parent'] +");"
                for sub in sm['substates']:
                    if state == sub['parent']:
                        if sub['parallel'] is "parallel":
                            cod = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates, " + substates['parent'] +");"
                            break
                cog.outl(cod)
            if substates['contents']['initialstate'] != "none":
                cod += ", *" + substates['contents']['initialstate'][0]
                cod += " = new QState(" + substates['parent'][0] + ");"
                for sub in sm['substates']:
                    if state == sub['parent']:
                        if sub['parallel'] is "parallel":
                            cod = "<TABHERE>QState *" + state + " = new QState(QState::ParallelStates, " + substates['parent'] + ");"
                            break
                cog.outl(cod)
            if substates['contents']['finalstate'] != "none":
                cog.outl("<TABHERE>QFinalState *" + substates['contents']['finalstate'][0] + " = new QFinalState(" +substates['parent'][0] + ");")
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
    for state in sm['machine']['contents']['states']:
        cod = "<TABHERE>virtual void fun_" + state + "() = 0;"
        cog.outl(cod)
    if sm['machine']['contents']['initialstate'] != "none":
        cod = "<TABHERE>virtual void fun_" + sm['machine']['contents']['initialstate'][0] + "() = 0;"
        cog.outl(cod)
    if sm['machine']['contents']['finalstate'] != "none":
        cod = "<TABHERE>virtual void fun_" + sm['machine']['contents']['finalstate'][0] + "() = 0;"
        cog.outl(cod)
    if sm['substates'] != "none":
        for substates in sm['substates']:
            for state in substates['contents']['states']:
                cod = "<TABHERE>virtual void fun_" + state + "() = 0;"
                cog.outl(cod)
            if substates['contents']['initialstate'] != "none":
                cod = "<TABHERE>virtual void fun_" + substates['contents']['initialstate'][0] + "() = 0;"
                cog.outl(cod)
            if substates['contents']['finalstate'] != "none":
                cod = "<TABHERE>virtual void fun_" + substates['contents']['finalstate'][0] + "() = 0;"
                cog.outl(cod)
]]]
[[[end]]]
signals:
	void kill();
[[[cog
if component['statemachine'] != 'none':
    if sm['machine']['contents']['transition'] != "none":
        for transi in sm['machine']['contents']['transition']:
            cod = "<TABHERE>void "
            for dest in transi['dest']:
                aux = cod
                aux += transi['src'] + "to" + dest + "();"
                cog.outl(aux)
    if sm['substates']!="none":
        for substates in sm['substates']:
            if substates['contents']['transition'] != "none":
                for transi in substates['contents']['transition']:
                    cod = "<TABHERE>void "
                    for dest in transi['dest']:
                        aux = cod
                        aux += transi['src'] + "to" + dest + "();"
                        cog.outl(aux)
]]]
[[[end]]]
};

#endif