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
    component['statemachine'] = 'none'
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
pool = IDSLPool(theIDSLs, includeDirectories)
includeList = pool.rosImports()
rosTypes = pool.getRosTypes()


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
#include <stdint.h>
#include <qlog/qlog.h>

[[[cog
if component['gui'] != 'none':
	cog.outl("#if Qt5_FOUND") 
	cog.outl("<TABHERE>#include <QtWidgets>")
	cog.outl("#else")
	cog.outl("<TABHERE>#include <QtGui>")
	cog.outl("#endif") 
	cog.outl("#include <ui_mainUI.h>")
]]]
[[[end]]]
[[[cog
if sm is not None:
	cog.outl("#include <QStateMachine>")
	cog.outl("#include <QState>")
]]]
[[[end]]]
#include <CommonBehavior.h>

[[[cog
usingList = []
for imp in component['recursiveImports'] + component["iceInterfaces"]:
	name = imp.split('/')[-1].split('.')[0]
	if not name in usingList:
		usingList.append(name)
for name in usingList:
	cog.outl('#include <'+name+'.h>')


if component['usingROS'] == True:
	cog.outl('#include <ros/ros.h>')
	for include in includeList:
		cog.outl('#include <'+include+'.h>')
	srvIncludes = {}
	for imp in component['requires']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communicationIsIce(imp):
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						srvIncludes[mname] = '#include <'+module['name']+'ROS/'+mname+'.h>'
	for imp in component['implements']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communicationIsIce(imp):
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						srvIncludes[mname] = '#include <'+module['name']+'ROS/'+mname+'.h>'
	for srv in srvIncludes.values():
		cog.outl(srv)

try:
	if isAGM1Agent(component):
		cog.outl("#include <agm.h>")
	if isAGM2Agent(component):
		cog.outl("#include <AGM2.h>")
		cog.outl("#include <agm2.h>")
except:
	pass
]]]
[[[end]]]

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
[[[cog
usingList = []
for imp in component['recursiveImports'] + component["iceInterfaces"]:
	name = imp.split('/')[-1].split('.')[0]
	if not name in usingList:
		usingList.append(name)
for name in usingList:
	cog.outl("using namespace RoboComp"+name+";")
]]]
[[[end]]]

[[[cog
if component['language'].lower() == 'cpp':
	cog.outl("typedef map <string,::IceProxy::Ice::Object*> MapPrx;")
else:
	proxy_list = []
	for name in component['requires'] + component['publishes']:
		proxy_list.append("RoboComp" + name + "::" + name + "PrxPtr")
	cog.outl("using TuplePrx = std::tuple<" + ",".join(proxy_list) + ">;")
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

[[[cog
if component['usingROS'] == True:
	#CREANDO CLASES PARA LOS PUBLISHERS
	for imp in component['publishes']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			theIdsl = pool.IDSLsModule(module)
			idsl = IDSLParsing.fromFileIDSL(theIdsl)
			cog.outl("<TABHERE>//class for rosPublisher")
			cog.outl("class Publisher"+nname+"\n{\npublic:")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						cog.outl("<TABHERE>ros::Publisher pub_"+mname+";")
			cog.outl("<TABHERE>Publisher"+nname+"(ros::NodeHandle *node)\n<TABHERE>{")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						for p in method['params']:
							s = "\""+mname+"\""
							if p['type'] in ('float','int'):
								cog.outl("<TABHERE><TABHERE>pub_"+mname+" = node->advertise<std_msgs::"+p['type'].capitalize()+"32>(node->resolveName("+s+"), 1000);")
							elif p['type'] in ('uint8','uint16','uint32','uint64'):
								cog.outl("<TABHERE><TABHERE>pub_"+mname+" = node->advertise<std_msgs::UInt"+p['type'].split('t')[1]+"32>(node->resolveName("+s+"), 1000);")
							elif p['type'] in rosTypes:
								cog.outl("<TABHERE><TABHERE>pub_"+mname+" = node->advertise<std_msgs::"+p['type'].capitalize()+">(node->resolveName("+s+"), 1000);")
							elif '::' in p['type']:
								cog.outl("<TABHERE><TABHERE>pub_"+mname+" = node->advertise<"+p['type']+">(node->resolveName("+s+"), 1000);")
							else:
								cog.outl("<TABHERE><TABHERE>pub_"+mname+" = node->advertise<"+module['name']+"ROS::"+p['type']+">(node->resolveName("+s+"), 1000);")
			cog.outl("<TABHERE>}")
			cog.outl("<TABHERE>~Publisher"+nname+"(){}")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						for p in method['params']:
							if p['type'] in ('float','int'):
								cog.outl("<TABHERE>void "+mname+"(std_msgs::"+p['type'].capitalize()+"32 "+p['name']+")")
								cog.outl("<TABHERE>{\n<TABHERE><TABHERE>pub_"+mname+".publish("+p['name']+");")
								cog.outl("<TABHERE>}")
							elif p['type'] in ('uint8','uint16','uint32','uint64'):
								cog.outl("<TABHERE>void "+mname+"(std_msgs::UInt"+p['type'].split('t')[1]+" "+p['name']+")")
								cog.outl("<TABHERE>{\n<TABHERE><TABHERE>pub_"+mname+".publish("+p['name']+");")
								cog.outl("<TABHERE>}")
							elif p['type'] in rosTypes:
								cog.outl("<TABHERE>void "+mname+"(std_msgs::"+p['type'].capitalize()+" "+p['name']+")")
								cog.outl("<TABHERE>{\n<TABHERE><TABHERE>pub_"+mname+".publish("+p['name']+");")
								cog.outl("<TABHERE>}")
							elif '::' in p['type']:
								cog.outl("<TABHERE>void "+mname+"("+p['type'].replace("::","ROS::")+" "+p['name']+")")
								cog.outl("<TABHERE>{\n<TABHERE><TABHERE>pub_"+mname+".publish("+p['name']+");")
								cog.outl("<TABHERE>}")
							else:
								cog.outl("<TABHERE>void "+mname+"("+module['name']+"ROS::"+p['type']+" "+p['name']+")")
								cog.outl("<TABHERE>{\n<TABHERE><TABHERE>pub_"+mname+".publish("+p['name']+");")
								cog.outl("<TABHERE>}")
			cog.outl("};")

	#CREANDO CLASES PARA LOS REQUIRES
	for imp in component['requires']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			cog.outl("<TABHERE>//class for rosServiceClient")
			cog.outl("class ServiceClient"+nname+"\n{\npublic:")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						cog.outl("<TABHERE>ros::ServiceClient srv_"+mname+";")
			cog.outl("<TABHERE>ServiceClient"+nname+"(ros::NodeHandle *node)\n<TABHERE>{")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						s = "\""+mname+"\""
						cog.outl("<TABHERE><TABHERE>srv_"+mname+" = node->serviceClient<"+module['name']+"ROS::"+mname+">(node->resolveName("+s+"), 1000);")
			cog.outl("<TABHERE>}")
			cog.outl("<TABHERE>~ServiceClient"+nname+"(){}")
			theIdsl = pool.IDSLsModule(module)
			idsl = IDSLParsing.fromFileIDSL(theIdsl)
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						methodDef     = "<TABHERE>bool "+mname+"("
						methodContent ="<TABHERE>{\n<TABHERE><TABHERE>"+ module['name']+"ROS::"+mname+" srv;\n"
						firstParam = True
						for p in method['params']:
							for im in idsl['module']['contents']:
								#obtener todos los campos del struct y hacer la asignacion
								if firstParam:
									if im['name'] == p['type'] and im['type'] == 'struct':
										for campos in im['structIdentifiers']:
											methodContent +="<TABHERE><TABHERE>srv.request."+p['name']+"."+campos['identifier']+" = "+p['name']+"."+campos['identifier']+";\n"
								else:
									if im['name'] == p['type'] and im['type'] == 'struct':
										for campos in im['structIdentifiers']:
											methodContent +="<TABHERE><TABHERE><TABHERE>"+p['name']+"."+campos['identifier']+" = srv.response."+p['name']+"."+campos['identifier']+";\n"
							if firstParam:
								if p['type'] in ('float','int'):
									methodDef     += "std_msgs::"+p['type'].capitalize()+"32 "+p['name']+", "
									methodContent +="<TABHERE><TABHERE>srv.request."+p['name']+" = "+p['name']+".data;\n"
								elif p['type'] in ('uint8','uint16','uint32','uint64'):
									methodDef     += "std_msgs::UInt"+p['type'].split('t')[1]+" "+p['name']+", "
									methodContent +="<TABHERE><TABHERE>srv.request."+p['name']+" = "+p['name']+".data;\n"
								elif p['type'] in rosTypes:
									methodDef     += "std_msgs::"+p['type'].capitalize()+" "+p['name']+", "
									methodContent +="<TABHERE><TABHERE>srv.request."+p['name']+" = "+p['name']+".data;\n"
								elif '::' in p['type']:
									methodDef     += p['type'].replace("::","ROS::")+" "+p['name']+", "
									methodContent +="<TABHERE><TABHERE>srv.request."+p['name']+" = "+p['name']+";\n"
								else:
									methodDef     += module['name']+"ROS::"+p['type']+" "+p['name']+", "
								methodContent += "<TABHERE><TABHERE>if(srv_"+mname+".call(srv))\n<TABHERE><TABHERE>{\n"
								firstParam = False
							else:
								firstParam = True
								if p['type'] in ('float','int'):
									methodDef     += "std_msgs::"+p['type'].capitalize()+"32 &"+p['name']+") "
									methodContent += "<TABHERE><TABHERE><TABHERE>"+p['name']+".data = srv.response."+p['name']+";\n"
								elif p['type'] in ('uint8','uint16','uint32','uint64'):
									methodDef     += "std_msgs::UInt"+p['type'].split('t')[1]+" &"+p['name']+") "
									methodContent += "<TABHERE><TABHERE><TABHERE>"+p['name']+".data = srv.response."+p['name']+";\n"
								elif p['type'] in rosTypes:
									methodDef     += "std_msgs::"+p['type'].capitalize()+" &"+p['name']+") "
									methodContent += "<TABHERE><TABHERE><TABHERE>"+p['name']+".data = srv.response."+p['name']+";\n"
								elif '::' in p['type']:
									methodDef     += p['type'].replace("::","ROS::")+" "&+p['name']+") "
									methodContent += "<TABHERE><TABHERE><TABHERE>"+p['name']+" = srv.response."+p['name']+";\n"
								else:
									methodDef     += module['name']+"ROS::"+p['type']+" &"+p['name']+") "
								methodContent += "<TABHERE><TABHERE><TABHERE>return true;\n<TABHERE><TABHERE>}\n<TABHERE><TABHERE>return false;"
						cog.outl(methodDef)
						cog.outl(methodContent)
						cog.outl("<TABHERE>}")
			cog.outl("};")
]]]
[[[end]]]
class GenericWorker :
[[[cog
if component['gui'] != 'none':
	cog.outl("#ifdef USE_QTGUI\n<TABHERE>public " + component['gui'][1] + ", public Ui_guiDlg\n#else\n<TABHERE>public QObject\n #endif")
else:
	cog.outl("public QObject")
]]]
[[[end]]]
{
Q_OBJECT
public:
[[[cog
if component['language'].lower() == 'cpp':
	cog.outl("<TABHERE>GenericWorker(MapPrx& mprx);")
else:
	cog.outl("<TABHERE>GenericWorker(TuplePrx tprx);")
]]]
[[[end]]]
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
except:
	pass

]]]
[[[end]]]


[[[cog
for name, num in getNameNumber(component['requires']):
	if communicationIsIce(name):
		if component['language'].lower() == "cpp":
			cog.outl('<TABHERE>'+name+'Prx '+name.lower()+num +'_proxy;')
		else:
			cog.outl('<TABHERE>'+name+'PrxPtr '+name.lower()+num +'_proxy;')

for name, num in getNameNumber(component['publishes']):
	if communicationIsIce(name):
		if component['language'].lower() == "cpp":
			cog.outl('<TABHERE>'+name+'Prx '+name.lower()+num +'_pubproxy;')
		else:
			cog.outl('<TABHERE>'+name+'PrxPtr '+name.lower()+num +'_pubproxy;')

]]]
[[[end]]]

[[[cog
if 'implements' in component:
	for impa in component['implements']:
		if type(impa) == str:
			imp = impa
		else:
			imp = impa[0]
		module = pool.moduleProviding(imp)
		for interface in module['interfaces']:
			if interface['name'] == imp:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					paramStrA = ''
					if communicationIsIce(impa):
						for p in method['params']:
							# delim
							if paramStrA == '': delim = ''
							else: delim = ', '
							# decorator
							ampersand = '&'
							if p['decorator'] == 'out':
								const = ''
							else:
								if component['language'].lower() == "cpp":
									const = 'const '
								else:
									const = ''
									ampersand = ''
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl("<TABHERE>virtual " + method['return'] + ' ' + interface['name'] + "_" + method['name'] + '(' + paramStrA + ") = 0;")
					else:
						paramStrA = module['name'] +"ROS::"+method['name']+"::Request &req, "+module['name']+"ROS::"+method['name']+"::Response &res"
						if imp in component['iceInterfaces']:
							cog.outl("<TABHERE>virtual bool ROS" + method['name'] + '(' + paramStrA + ") = 0;")
						else:
							cog.outl("<TABHERE>virtual bool " + method['name'] + '(' + paramStrA + ") = 0;")
if 'subscribesTo' in component:
	for impa in component['subscribesTo']:
		if type(impa) == str:
			imp = impa
		else:
			imp = impa[0]
		module = pool.moduleProviding(imp)
		if module == None:
			print ('\nCan\'t find module providing', imp, '\n')
			sys.exit(-1)
		for interface in module['interfaces']:
			if interface['name'] == imp:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					paramStrA = ''
					if communicationIsIce(impa):
						for p in method['params']:
							# delim
							if paramStrA == '': delim = ''
							else: delim = ', '
							# decorator
							ampersand = '&'
							if p['decorator'] == 'out':
								const = ''
							else:
								if component['language'].lower() == "cpp":
									const = 'const '
								else:
									const = ''
									ampersand = ''
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl("<TABHERE>virtual " + method['return'] + ' ' + interface['name'] + "_" + method['name'] + '(' + paramStrA + ") = 0;")
					else:
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
								ampersand = ''
							if p['type'] in ('float','int'):
								p['type'] = "std_msgs::"+p['type'].capitalize()+"32"
							elif p['type'] in ('uint8','uint16','uint32','uint64'):
								p['type'] = "std_msgs::UInt"+p['type'].split('t')[1]
							elif p['type'] in rosTypes:
								p['type'] = "std_msgs::"+p['type'].capitalize()
							elif not '::' in p['type']:
								p['type'] = module['name']+"ROS::"+p['type']
							# STR
							paramStrA += delim + p['type'] + ' ' + p['name']
						if imp in component['iceInterfaces']:
							cog.outl("<TABHERE>virtual void ROS" + method['name'] + '(' + paramStrA + ") = 0;")
						else:
							cog.outl("<TABHERE>virtual void " + method['name'] + '(' + paramStrA + ") = 0;")

]]]
[[[end]]]

protected:
[[[cog
if sm is not None:
    codQState = ""
    codQStateMachine = ""
    lsstates = ""
    codQStateMachine = "<TABHERE>QStateMachine " + sm['machine']['name'] + ";\n"
    if sm['machine']['contents']['states'] is not "none":
        for state in sm['machine']['contents']['states']:
            aux = "<TABHERE>QState *" + state + "State = new QState();\n"
            lsstates += state +","
            if sm['substates'] is not "none":
                for substates in sm['substates']:
                    if state == substates['parent']:
                        if substates['parallel'] is "parallel":
                            aux = "<TABHERE>QState *" + state + "State = new QState(QState::ParallelStates);\n"
                            break
            codQState += aux
    if sm['machine']['contents']['initialstate'] != "none":
        state = sm['machine']['contents']['initialstate'][0]
        aux = "<TABHERE>QState *" + state + "State = new QState();\n"
        lsstates += state +","
        if sm['substates'] is not "none":
            for substates in sm['substates']:
                if state == substates['parent']:
                    if substates['parallel'] is "parallel":
                        aux = "<TABHERE>QState *" + state + "State = new QState(QState::ParallelStates);\n"
                        break
        codQState += aux


    if sm['machine']['contents']['finalstate'] != "none":
        state = sm['machine']['contents']['finalstate'][0]
        codQState +="<TABHERE>QFinalState *" + state + "State = new QFinalState();\n"
        lsstates += state +","

    if sm['substates'] != "none":
        for substates in sm['substates']:
            if substates['contents']['states'] is not "none":
                for state in substates['contents']['states']:
                    aux = "<TABHERE>QState *" + state + "State = new QState(" + substates['parent'] +"State);\n"
                    lsstates += state +","
                    for sub in sm['substates']:
                        if state == sub['parent']:
                            if sub['parallel'] is "parallel":
                                aux = "<TABHERE>QState *" + state + "State = new QState(QState::ParallelStates, " + substates['parent'] +"State);\n"
                                break
                    codQState += aux
            if substates['contents']['initialstate'] != "none":
                aux = "<TABHERE>QState *" + substates['contents']['initialstate'] + "State = new QState(" + substates['parent'] + "State);\n"
                lsstates += state +","
                for sub in sm['substates']:
                    if state == sub['parent']:
                        if sub['parallel'] is "parallel":
                            aux = "<TABHERE>QState *" + state + "State = new QState(QState::ParallelStates, " + substates['parent'] + "State);\n"
                            break
                codQState += aux
            if substates['contents']['finalstate'] != "none":
                codQState += "<TABHERE>QFinalState *" + substates['contents']['finalstate'] + "State = new QFinalState(" +substates['parent'] + "State);\n"
                lsstates += state +","



    cog.outl("//State Machine")
    cog.outl(codQStateMachine)
    cog.outl(codQState)
    cog.outl("//-------------------------")


]]]
[[[end]]]

	QTimer timer;
	int Period;
[[[cog
if component['usingROS'] == True:
	cog.outl("<TABHERE>ros::NodeHandle node;")
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
					method = interface['methods'][mname]
					cog.outl("<TABHERE>ros::Subscriber "+nname+"_"+mname+";")
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
					method = interface['methods'][mname]
					cog.outl("<TABHERE>ros::ServiceServer "+nname+"_"+mname+";")
if 'publishes' in component:
	for publish in component['publishes']:
		pubs = publish
		while type(pubs) != type(''):
			pubs = pubs[0]
		if not communicationIsIce(publish):
			if pubs in component['iceInterfaces']:
				cog.outl("<TABHERE>Publisher"+pubs+" *"+pubs.lower()+"_rosproxy;")
			else:
				cog.outl("<TABHERE>Publisher"+pubs+" *"+pubs.lower()+"_proxy;")
if 'requires' in component:
	for require in component['requires']:
		req = require
		while type(req) != type(''):
			req = req[0]
		if not communicationIsIce(require):
			if req in component['iceInterfaces']:
				cog.outl("<TABHERE>ServiceClient"+req+" *"+req.lower()+"_rosproxy;")
			else:
				cog.outl("<TABHERE>ServiceClient"+req+" *"+req.lower()+"_proxy;")
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>bool active;")
		cog.outl("<TABHERE>AGMModel::SPtr worldModel;")
		cog.outl("<TABHERE>BehaviorParameters p;")
		cog.outl("<TABHERE>ParameterMap params;")
		cog.outl("<TABHERE>int iter;")
		cog.outl("<TABHERE>bool setParametersAndPossibleActivation(const RoboCompAGMCommonBehavior::ParameterMap &prs, bool &reactivated);")
		cog.outl("<TABHERE>RoboCompPlanning::Action createAction(std::string s);")
except:
	pass

]]]
[[[end]]]

private:


public slots:
[[[cog
if sm is not None:
    sm_virtual_methods = ""
    if sm['machine']['contents']['states'] is not "none":
        for state in sm['machine']['contents']['states']:
            sm_virtual_methods += "<TABHERE>virtual void sm_" + state + "() = 0;\n"
    if sm['machine']['contents']['initialstate'] != "none":
        sm_virtual_methods += "<TABHERE>virtual void sm_" + sm['machine']['contents']['initialstate'][0] + "() = 0;\n"
    if sm['machine']['contents']['finalstate'] != "none":
        sm_virtual_methods += "<TABHERE>virtual void sm_" + sm['machine']['contents']['finalstate'][0] + "() = 0;\n"
    if sm['substates'] != "none":
        for substates in sm['substates']:
            if substates['contents']['states'] is not "none":
                for state in substates['contents']['states']:
                    sm_virtual_methods += "<TABHERE>virtual void sm_" + state + "() = 0;\n"
            if substates['contents']['initialstate'] != "none":
                sm_virtual_methods += "<TABHERE>virtual void sm_" + substates['contents']['initialstate'] + "() = 0;\n"
            if substates['contents']['finalstate'] != "none":
                sm_virtual_methods += "<TABHERE>virtual void sm_" + substates['contents']['finalstate'] + "() = 0;\n"
    cog.outl("//Slots funtion State Machine")
    cog.outl(sm_virtual_methods)
    cog.outl("//-------------------------")

if (sm is not None and sm['machine']['default'] is True) or component['statemachine'] == 'none':
    cog.outl("<TABHERE>virtual void compute() = 0;")
]]]
[[[end]]]
    virtual void initialize(int period) = 0;
	
signals:
	void kill();
[[[cog
if sm is not None:
    codsignals = ""
    if sm['machine']['contents']['transitions'] != "none":
        for transi in sm['machine']['contents']['transitions']:
            for dest in transi['dest']:
                codsignals += "<TABHERE>void " +  transi['src'] + "to" + dest + "();\n"
    if sm['substates']!="none":
        for substates in sm['substates']:
            if substates['contents']['transitions'] != "none":
                for transi in substates['contents']['transitions']:
                    for dest in transi['dest']:
                        codsignals += "<TABHERE>void " + transi['src'] + "to" + dest + "();\n"
    cog.outl("//Signals for State Machine")
    cog.outl(codsignals)
    cog.outl("//-------------------------")
]]]
[[[end]]]
};

#endif
