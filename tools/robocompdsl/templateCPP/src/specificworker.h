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
from parseSMDSL import *
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

/**
       \brief
       @author authorname
*/

[[[cog
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("// THIS IS AN AGENT")
except:
	pass
]]]
[[[end]]]


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
[[[cog
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>#include <osgviewer/osgview.h>")
	cog.outl("<TABHERE>#include <innermodel/innermodelviewer.h>")
	cog.outl("#endif")
]]]
[[[end]]]

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
[[[cog
	if component['language'].lower() == 'cpp':
		cog.outl("<TABHERE>SpecificWorker(MapPrx& mprx);")
	else:
		cog.outl("<TABHERE>SpecificWorker(TuplePrx tprx);")
]]]
[[[end]]]
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

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
						cog.outl("<TABHERE>" + method['return'] + ' ' +interface['name'] + "_" + method['name'] + '(' + paramStrA + ");")
					else:
						paramStrA = module['name'] +"ROS::"+method['name']+"::Request &req, "+module['name']+"ROS::"+method['name']+"::Response &res"
						if imp in component['iceInterfaces']:
							cog.outl("<TABHERE>bool ROS" + method['name'] + '(' + paramStrA + ");")
						else:
							cog.outl("<TABHERE>bool " + method['name'] + '(' + paramStrA + ");")

if 'subscribesTo' in component:
	for impa in component['subscribesTo']:
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
						cog.outl("<TABHERE>" + method['return'] + ' ' +interface['name'] + "_" + method['name'] + '(' + paramStrA + ");")
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
							cog.outl("<TABHERE>void ROS" + method['name'] + '(' + paramStrA + ");")
						else:
							cog.outl("<TABHERE>void " + method['name'] + '(' + paramStrA + ");")

]]]
[[[end]]]

public slots:
[[[cog
if (sm is not None and sm['machine']['default'] is True) or component['statemachine'] is None:
	cog.outl("<TABHERE>void compute();")
]]]
[[[end]]]
	void initialize(int period);
[[[cog
if component['statemachine'] is not None:
    sm_specification = ""
    if sm['machine']['contents']['states'] is not None:
        for state in sm['machine']['contents']['states']:
            sm_specification += "<TABHERE>void sm_" + state + "();\n"
    if sm['machine']['contents']['initialstate'] is not None:
        sm_specification += "<TABHERE>void sm_" + sm['machine']['contents']['initialstate'] + "();\n"
    if sm['machine']['contents']['finalstate'] is not None:
        sm_specification += "<TABHERE>void sm_" + sm['machine']['contents']['finalstate'] + "();\n"
    if sm['substates'] is not None:
        for substates in sm['substates']:
            if substates['contents']['states'] is not None:
                for state in substates['contents']['states']:
                    sm_specification += "<TABHERE>void sm_" + state + "();\n"
            if substates['contents']['initialstate'] is not None:
                sm_specification += "<TABHERE>void sm_" + substates['contents']['initialstate'] + "();\n"
            if substates['contents']['finalstate'] is not None:
                sm_specification += "<TABHERE>void sm_" + substates['contents']['finalstate'] + "();\n"
    cog.outl("//Specification slot methods State Machine")
    cog.outl(sm_specification)
    cog.outl("//--------------------")
]]]
[[[end]]]
private:
	std::shared_ptr<InnerModel> innerModel;
[[[cog
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>OsgView *osgView;")
	cog.outl("<TABHERE>InnerModelViewer *innerModelViewer;")
	cog.outl("#endif")
try:
	if isAGM1Agent(component):
		cog.outl("<TABHERE>std::string action;")
		cog.outl("<TABHERE>ParameterMap params;")
		cog.outl("<TABHERE>AGMModel::SPtr worldModel;")
		cog.outl("<TABHERE>bool active;")
		if 'innermodelviewer' in [ x.lower() for x in component['options'] ]:
			cog.outl("<TABHERE>void regenerateInnerModelViewer();")
		cog.outl("<TABHERE>bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);")
		cog.outl("<TABHERE>void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);")
	elif isAGM2Agent(component):
		cog.outl("<TABHERE>std::string action;")
		cog.outl("<TABHERE>AGMModel::SPtr worldModel;")
		cog.outl("<TABHERE>bool active;")
except:
	pass

]]]
[[[end]]]

};

#endif
