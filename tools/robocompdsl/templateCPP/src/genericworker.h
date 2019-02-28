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

#include <CommonBehavior.h>

[[[cog
for imp in component['recursiveImports']:
	incl = imp.split('/')[-1].split('.')[0]
	cog.outl('#include <'+incl+'.h>')


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
pool = IDSLPool(theIDSLs, includeDirectories)
for m in pool.modulePool:
	rosModule = False
	for imp in component['subscribesTo']+component['publishes']+component['implements']+component['requires']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communicationIsIce(imp):
			if im not in component['iceInterfaces']:
				rosModule = True
	if rosModule == False:
		cog.outl("using namespace "+pool.modulePool[m]['name']+";")

]]]
[[[end]]]

[[[cog
if component['language'].lower() == 'cpp':
	cog.outl("typedef map <string,::IceProxy::Ice::Object*> MapPrx;")
else:
	proxy_list = []
	for imp in component['publishes'] + component['requires']:
		if type(imp) == str:
			name = imp
		else:
			name = imp[0]
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
for namea, num in getNameNumber(component['publishes']) + getNameNumber(component['requires']):
	if type(namea) == str:
		name = namea
	else:
		name = namea[0]
	if communicationIsIce(namea):
		if component['language'].lower() == "cpp":
			cog.outl('<TABHERE>'+name+'Prx '+name.lower()+num +'_proxy;')
		else:
			cog.outl('<TABHERE>'+name+'PrxPtr '+name.lower()+num +'_proxy;')
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
		cog.outl("<TABHERE>bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);")
		cog.outl("<TABHERE>RoboCompPlanning::Action createAction(std::string s);")
except:
	pass

]]]
[[[end]]]

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
signals:
	void kill();
};

#endif
