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
component = CDSLParsing.fromFile(theCDSL)
sm = SMDSLparsing.fromFile(component['statemachine'])
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
if component['useViewer'] == "true":
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
	SpecificWorker(MapPrx& mprx);	
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
								const = 'const '
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl("<TABHERE>" + method['return'] + ' ' + method['name'] + '(' + paramStrA + ");")
					else:
						paramStrA = module['name'] +"::"+method['name']+"::Request &req, "+module['name']+"::"+method['name']+"::Response &res"
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
								const = 'const '
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl("<TABHERE>" + method['return'] + ' ' + method['name'] + '(' + paramStrA + ");")
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
							if p['type'] in ('float','int','uint'):
								p['type'] = "std_msgs::"+p['type'].capitalize()+"32"
							elif p['type'] == 'string':
								p['type'] = "std_msgs::String"
							elif not '::' in p['type']:
								p['type'] = module['name']+"::"+p['type']
							# STR
							paramStrA += delim + p['type'] + ' ' + p['name']
						cog.outl("<TABHERE>void " + method['name'] + '(' + paramStrA + ");")

]]]
[[[end]]]

public slots:

[[[cog
if component['statemachine'] != 'none':
    specificationfun = ""
    if sm['machine']['contents']['states'] is not "none":
        for state in sm['machine']['contents']['states']:
            specificationfun += "<TABHERE>void fun_" + state + "();\n"
    if sm['machine']['contents']['initialstate'] != "none":
        specificationfun += "<TABHERE>void fun_" + sm['machine']['contents']['initialstate'][0] + "();\n"
    if sm['machine']['contents']['finalstate'] != "none":
        specificationfun += "<TABHERE>void fun_" + sm['machine']['contents']['finalstate'][0] + "();\n"
    if sm['substates'] != "none":
        for substates in sm['substates']:
            if substates['contents']['states'] is not "none":
                for state in substates['contents']['states']:
                    specificationfun += "<TABHERE>void fun_" + state + "();\n"
            if substates['contents']['initialstate'] != "none":
                specificationfun += "<TABHERE>void fun_" + substates['contents']['initialstate'] + "();\n"
            if substates['contents']['finalstate'] != "none":
                specificationfun += "<TABHERE>void fun_" + substates['contents']['finalstate'] + "();\n"
    cog.outl("//Specification slot funtions State Machine")
    cog.outl(specificationfun)
    cog.outl("//--------------------")
else:
    cog.outl("void compute();")
]]]
[[[end]]]
private:
	InnerModel *innerModel;
[[[cog
if component['useViewer'] == "true":
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>OsgView *osgView;")
	cog.outl("<TABHERE>InnerModelViewer *imv;")
	cog.outl("#endif")
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>std::string action;")
		cog.outl("<TABHERE>ParameterMap params;")
		cog.outl("<TABHERE>AGMModel::SPtr worldModel;")
		cog.outl("<TABHERE>InnerModel *innerModel;")
		cog.outl("<TABHERE>bool active;")
		cog.outl("<TABHERE>bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);")
		cog.outl("<TABHERE>void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);")
except:
	pass

]]]
[[[end]]]

private slots:

	
};

#endif

