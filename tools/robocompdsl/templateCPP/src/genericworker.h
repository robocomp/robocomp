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
component = CDSLParsing.fromFile(theCDSL)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
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

#include <CommonBehavior.h>

[[[cog
usingROS = False
includeMap = {}
if 'subscribesTo' in component:
	for subscribe in component['subscribesTo']:
		subs = subscribe
		while type(subs) != type(''):
			subs = subs[0]
		if not communicationIsIce(subscribe):
			usingROS = True
			includeMap[subs] = '#include <std_msgs/'+subs+'.h>'

if 'publishes' in component:
	for publish in component['publishes']:
		pubs = publish
		while type(pubs) != type(''):
			pubs = pubs[0]
		if not communicationIsIce(publish):
			usingROS = True
			includeMap[pubs] = '#include <std_msgs/'+pubs+'.h>'

if usingROS:
	cog.outl('#include <ros/ros.h>')
	for s in includeMap: 
		cog.outl(includeMap[s])
]]]
[[[end]]]

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
	if communicationIsIce(name):
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
		if communicationIsIce(imp):
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
			cog.outl("<TABHERE>virtual void ros" + nname + '(const std_msgs::' + nname + "::ConstPtr& recv" + nname + ") = 0;")
	

				
]]]
[[[end]]]

[[[cog
if 'subscribesTo' in component:
	for subscribe in component['subscribesTo']:
		subs = subscribe
		while type(subs) != type(''):
			subs = subs[0]
		if not communicationIsIce(subscribe):
			cog.outl('<TABHERE>void setROSSub'+subs+'(std::string s, int max);')

if 'publishes' in component:
	for publish in component['publishes']:
		pubs = publish
		while type(pubs) != type(''):
			pubs = pubs[0]
		if not communicationIsIce(publish):
			cog.outl('<TABHERE>void setROSPub'+pubs+'(std::string s, int max);')
]]]
[[[end]]]

protected:
	QTimer timer;
	int Period;
[[[cog
if 'publishes' in component:
	for publish in component['publishes']:
		pubs = publish
		while type(pubs) != type(''):
			pubs = pubs[0]
		if not communicationIsIce(publish):
			cog.outl('<TABHERE>ros::NodeHandle npub'+pubs+';')
			cog.outl('<TABHERE>ros::Publisher pub'+pubs+';')
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

private:
[[[cog
if 'subscribesTo' in component:
	for subscribe in component['subscribesTo']:
		subs = subscribe
		while type(subs) != type(''):
			subs = subs[0]
		if not communicationIsIce(subscribe):
			cog.outl('<TABHERE>ros::NodeHandle nsub'+subs+';')
			cog.outl('<TABHERE>ros::Subscriber sub'+subs+';')
]]]
[[[end]]]

public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif