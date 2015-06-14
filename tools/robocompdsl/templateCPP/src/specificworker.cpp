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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

[[[cog

try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>active = false;")
		cog.outl("<TABHERE>worldModel = AGMModel::SPtr(new AGMModel());")
		cog.outl("<TABHERE>worldModel->name = "+"\"worldModel\";")
except:
	pass

]]]
[[[end]]]
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}


[[[cog
ll = []
if 'implements'   in component: ll += component['implements']
if 'subscribesTo' in component: ll += component['subscribesTo']
for imp in ll:
	module = pool.moduleProviding(imp)
	for interface in module['interfaces']:
		if interface['name'] == imp:
			for mname in interface['methods']:
				method = interface['methods'][mname]
				paramStrA = ''
				for p in method['params']:
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
					paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
				bodyCode=""
				###################################### 
				#code for subscribesTo AGMExecutiveTopic
				###################################### 
				if method['name'] == 'structuralChange':
					bodyCode = "<TABHERE>mutex->lock();\n <TABHERE>AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);\n <TABHERE>mutex->unlock();"
				if method['name'] == 'symbolUpdated' or method['name'] == 'edgeUpdated':
					bodyCode = "<TABHERE>mutex->lock();\n <TABHERE>AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n <TABHERE>mutex->unlock();"
					
				###################################### 
				#code for implements AGMCommonBehavior.
				###################################### 
				if method['name'] == 'activateAgent':
					bodyCode = "/*<TABHERE>bool activated = false;\n<TABHERE>if (setParametersAndPossibleActivation(prs, activated))\n<TABHERE>{\n<TABHERE><TABHERE>if (not activated)\n<TABHERE><TABHERE>{\n<TABHERE><TABHERE><TABHERE>return activate(p);\n<TABHERE><TABHERE>}\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>return false;\n<TABHERE>}\n<TABHERE>return true;*/"
					
				if method['name'] == 'deactivateAgent':
					bodyCode = "//<TABHERE>return deactivate();"
					
				if method['name'] == 'getAgentState':
					bodyCode = "<TABHERE>StateStruct s;\n<TABHERE>/*if (isActive())\n<TABHERE>{\n<TABHERE><TABHERE>s.state = Running;\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>s.state = Stopped;\n<TABHERE>}\n<TABHERE>s.info = p.action.name;*/\n<TABHERE>return s;"
					
				if method['name'] == 'getAgentParameters':
					bodyCode = "<TABHERE>return params;"
					
				if method['name'] == 'setAgentParameters':
					bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>//return setParametersAndPossibleActivation(prs, activated);"
					
				if method['name'] == 'uptimeAgent':
					bodyCode = "<TABHERE>return 0;"
				if method['name'] == 'reloadConfigAgent':
					bodyCode = "<TABHERE>return true;"
					
				cog.outl(method['return'] + ' SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
]]]
[[[end]]]




