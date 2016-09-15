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

def bodyCodeFromName(name, component):
	bodyCode=""
	if isAGM1Agent(component):
		#######################################################
		# code to implement subscription to AGMExecutiveTopic #
		#######################################################
		if name == 'symbolsUpdated' or name == 'symbolsUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto n : modification)\n\t\tAGMModelConverter::includeIceModificationInInternalModel(n, worldModel);\n"
		elif name == 'edgesUpdated' or name == 'edgeUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto e : modification)\n\t{\n\t\tAGMModelConverter::includeIceModificationInInternalModel(e, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, e, innerModelViewer->innerModel);\n\t}\n"
		if name == 'symbolUpdated' or name == 'edgeUpdated' or name == 'symbolsUpdated' or name == 'edgesUpdated':
			bodyCode = "<TABHERE>QMutexLocker locker(mutex);\n<TABHERE>AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n \n<TABHERE>delete innerModel;\n<TABHERE>innerModel = AGMInner::extractInnerModel(worldModel);"
		elif name == 'structuralChange':
			bodyCode = "<TABHERE>mutex->lock();\n <TABHERE>AGMModelConverter::fromIceToInternal(w, worldModel);\n \n<TABHERE>delete innerModel;\n<TABHERE>innerModel = AGMInner::extractInnerModel(worldModel);\n<TABHERE>mutex->unlock();"
		#######################################
		# code to implement AGMCommonBehavior #
		#######################################
		elif name == 'activateAgent':
			bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>if (setParametersAndPossibleActivation(prs, activated))\n<TABHERE>{\n<TABHERE><TABHERE>if (not activated)\n<TABHERE><TABHERE>{\n<TABHERE><TABHERE><TABHERE>return activate(p);\n<TABHERE><TABHERE>}\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>return false;\n<TABHERE>}\n<TABHERE>return true;"
		elif name == 'deactivateAgent':
			bodyCode = "<TABHERE>return deactivate();"
		elif name == 'getAgentState':
			bodyCode = "<TABHERE>StateStruct s;\n<TABHERE>if (isActive())\n<TABHERE>{\n<TABHERE><TABHERE>s.state = Running;\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>s.state = Stopped;\n<TABHERE>}\n<TABHERE>s.info = p.action.name;\n<TABHERE>return s;"
		elif name == 'getAgentParameters':
			bodyCode = "<TABHERE>return params;"
		elif name == 'setAgentParameters':
			bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>return setParametersAndPossibleActivation(prs, activated);"
		elif name == 'uptimeAgent':
			bodyCode = "<TABHERE>return 0;"
		elif name == 'reloadConfigAgent':
			bodyCode = "<TABHERE>return true;"

	elif isAGM2Agent(component):
		mdlw = 'Ice'
		if isAGM2AgentROS(component):
			mdlw = 'ROS'
		elif name == 'symbolsUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto n : modification"
			if mdlw == 'ROS':
				bodyCode += ".NodeSequence"
			bodyCode += ")\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(n, worldModel);\n"
		elif name == 'edgesUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tfor (auto e : modification"
			if mdlw == 'ROS':
				bodyCode += ".EdgeSequence"
			bodyCode += ")\n\t{\n\t\tAGMModelConverter::include" + mdlw + "ModificationInInternalModel(e, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, e, innerModelViewer->innerModel);\n\t}\n"
		elif name == 'structuralChange':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::from" + mdlw + "ToInternal(w, worldModel);\n\t\n\tInnerModel *newIM = AGMInner::extractInnerModel(worldModel);\n"
			if 'innermodelviewer' in component['options']:
				bodyCode += "\tif (innerModelViewer)\n\t{\n\t\tosgView->getRootGroup()->removeChild(innerModelViewer);\n\t\tdelete innerModel;\n\t}\n"
			bodyCode += "\tinnerModel = newIM;\n"
			if 'innermodelviewer' in component['options']:
				bodyCode += "\tinnerModelViewer = new InnerModelViewer(innerModel, \"root\", osgView->getRootGroup(), true);\n"

		return bodyCode


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
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>innerModelViewer = NULL;")
	cog.outl("<TABHERE>osgView = new OsgView(this);")
	cog.outl("<TABHERE>osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;")
	cog.outl("<TABHERE>osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));")
	cog.outl("<TABHERE>osg::Vec3d center(osg::Vec3(0.,0.,-0.));")
	cog.outl("<TABHERE>osg::Vec3d up(osg::Vec3(0.,1.,0.));")
	cog.outl("<TABHERE>tb->setHomePosition(eye, center, up, true);")
	cog.outl("<TABHERE>tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));")
 	cog.outl("<TABHERE>osgView->setCameraManipulator(tb);")
	cog.outl("#endif")
try:
	if isAGM1Agent(component):
		cog.outl("<TABHERE>active = false;")
		cog.outl("<TABHERE>worldModel = AGMModel::SPtr(new AGMModel());")
		cog.outl("<TABHERE>worldModel->name = "+"\"worldModel\";")
		cog.outl("<TABHERE>innerModel = new InnerModel();")
	if isAGM2Agent(component):
		cog.outl("<TABHERE>active = false;")
		cog.outl("<TABHERE>worldModel = AGMModel::SPtr(new AGMModel());")
		cog.outl("<TABHERE>worldModel->name = "+"\"worldModel\";")
		cog.outl("<TABHERE>innerModel = new InnerModel();")
		
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
[[[cog
cog.outl("""//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }

""")
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>innerModelViewer = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);")
	cog.outl("#endif")
]]]
[[[end]]]	

	
	timer.start(Period);
	
[[[cog
try:
	if isAGM1Agent(component):
		cog.outl("<TABHERE>try")
		cog.outl("<TABHERE>{")
		cog.outl("<TABHERE><TABHERE>RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();")
		cog.outl("<TABHERE><TABHERE>structuralChange(w);")
		cog.outl("<TABHERE>}")
		cog.outl("<TABHERE>catch(...)")
		cog.outl("<TABHERE>{")
		cog.outl("<TABHERE><TABHERE>printf(\"The executive is probably not running, waiting for first AGM model publication...\");")
		cog.outl("<TABHERE>}")
	elif isAGM2Agent(component):
		cog.outl("// TODO: Here we should ask the DSR for the current model for initialization purposes.")

except:
	pass
]]]
[[[end]]]

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	
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
[[[cog
if component['usingROS'] == True:
	cog.outl("<TABHERE>ros::spinOnce();")
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>if (innerModelViewer) innerModelViewer->update();")
	cog.outl("<TABHERE>osgView->frame();")
	cog.outl("#endif")
]]]
[[[end]]]
}


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
					bodyCode = bodyCodeFromName(method['name'], component)
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
						cog.outl(method['return'] + ' SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
					else:
						paramStrA = module['name'] +"ROS::"+method['name']+"::Request &req, "+module['name']+"ROS::"+method['name']+"::Response &res"
						if imp in component['iceInterfaces']:
							cog.outl('bool SpecificWorker::ROS' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
						else:
							cog.outl('bool SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")

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
					bodyCode = bodyCodeFromName(method['name'], component)
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
						cog.outl(method['return'] + ' SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
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
							elif p['type'] in ('string', 'bool'):
								p['type'] = "std_msgs::"+p['type'].capitalize()
							elif not '::' in p['type']:
								p['type'] = module['name']+"ROS::"+p['type']
							# STR
							paramStrA += delim + p['type'] + ' ' + p['name']
						if imp in component['iceInterfaces']:
							cog.outl('void SpecificWorker::ROS' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
						else:
							cog.outl('void SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n"+bodyCode+"\n}\n")
]]]
[[[end]]]

[[[cog
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("""
bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\\n");

	return true;
}""")
		cog.outl ("""void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMModelPrinter::printWorld(newModel);""")
		agentName=component['name']
		cog.outl("<TABHERE><TABHERE>AGMMisc::publishModification(newModel, agmexecutive_proxy, \""+ agentName+"Agent\");")
		cog.outl ("""<TABHERE>}
	catch(...)
	{
		exit(1);
	}
}""")
		
		
except:
	pass
]]]
[[[end]]]




