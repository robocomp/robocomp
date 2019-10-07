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

def bodyCodeFromName(name, component):
	bodyCode=""
	if isAGM1Agent(component):
		#######################################################
		# code to implement subscription to AGMExecutiveTopic #
		#######################################################
		if name == 'symbolUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
		if name == 'symbolsUpdated':
			bodyCode = "\tQMutexLocker l(mutex);\n\tfor (auto modification : modifications)\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n"
		if name == 'edgeUpdated':
			bodyCode = "\tQMutexLocker locker(mutex);\n\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);\n"
		if name == 'edgesUpdated':
			bodyCode = "\tQMutexLocker lockIM(mutex);\n\tfor (auto modification : modifications)\n\t{\n\t\tAGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);\n\t\tAGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);\n\t}\n"
		if name == 'structuralChange':
			bodyCode = "<TABHERE>QMutexLocker lockIM(mutex);\n <TABHERE>AGMModelConverter::fromIceToInternal(w, worldModel);\n \n<TABHERE>delete innerModel;\n<TABHERE>innerModel = AGMInner::extractInnerModel(worldModel);"
			if 'innermodelviewer' in [ x.lower() for x in component['options'] ]:
				bodyCode += "\n<TABHERE>regenerateInnerModelViewer();"
		#######################################
		# code to implement AGMCommonBehavior #
		#######################################
		elif name == 'activateAgent':
			bodyCode = "<TABHERE>bool activated = false;\n<TABHERE>if (setParametersAndPossibleActivation(prs, activated))\n<TABHERE>{\n<TABHERE><TABHERE>if (not activated)\n<TABHERE><TABHERE>{\n<TABHERE><TABHERE><TABHERE>return activate(p);\n<TABHERE><TABHERE>}\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>return false;\n<TABHERE>}\n<TABHERE>return true;"
		elif name == 'deactivateAgent':
			bodyCode = "<TABHERE>return deactivate();"
		elif name == 'getAgentState':
			bodyCode = "<TABHERE>StateStruct s;\n<TABHERE>if (isActive())\n<TABHERE>{\n<TABHERE><TABHERE>s.state = RoboCompAGMCommonBehavior::StateEnum::Running;\n<TABHERE>}\n<TABHERE>else\n<TABHERE>{\n<TABHERE><TABHERE>s.state = RoboCompAGMCommonBehavior::StateEnum::Stopped;\n<TABHERE>}\n<TABHERE>s.info = p.action.name;\n<TABHERE>return s;"
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
[[[cog
    if component['language'].lower() == 'cpp':
        cog.outl("SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)")
    else:
        cog.outl("SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)")
]]]
[[[end]]]
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
	std::cout << "Destroying SpecificWorker" << std::endl;
[[[cog
if sm is not None and sm['machine']['default']:
	cog.outl("<TABHERE>emit t_compute_to_finalize();")
]]]
[[[end]]]
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
[[[cog
cog.outl("""//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

""")
if component['innermodelviewer']:
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>innerModel = std::make_shared<InnerModel>(); //InnerModel creation example")
	cog.outl("<TABHERE>innerModelViewer = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);")
	cog.outl("#endif")
]]]
[[[end]]]

[[[cog
if sm is not None:
    cog.outl("<TABHERE>" + sm['machine']['name'] + ".start();")
]]]
[[[end]]]
	

[[[cog
try:
	if isAGM1Agent(component):
		cog.outl("<TABHERE>try")
		cog.outl("<TABHERE>{")
		cog.outl("<TABHERE><TABHERE>RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();")
		cog.outl("<TABHERE><TABHERE>AGMExecutiveTopic_structuralChange(w);")
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

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);
[[[cog
if sm is not None and sm['machine']['default']:
    cog.outl("<TABHERE>emit this->t_initialize_to_compute();")
    ]]]
[[[end]]]

}

[[[cog
if (sm is not None and sm['machine']['default'] is True) or component['statemachine'] is None:
    cog.outl("void SpecificWorker::compute()")
    cog.outl("{")
    cog.outl("//computeCODE")
    cog.outl("//QMutexLocker locker(mutex);")
    cog.outl("//<TABHERE>try")
    cog.outl("//<TABHERE>{")
    cog.outl("//<TABHERE><TABHERE>camera_proxy->getYImage(0,img, cState, bState);")
    cog.outl("//<TABHERE><TABHERE>memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));")
    cog.outl("//<TABHERE><TABHERE>searchTags(image_gray);")
    cog.outl("//<TABHERE>}")
    cog.outl("//<TABHERE>catch(const Ice::Exception &e)")
    cog.outl("//<TABHERE>{")
    cog.outl("//<TABHERE><TABHERE>std::cout << \"Error reading from Camera\" << e << std::endl;")
    cog.outl("//<TABHERE>}")
    if component['usingROS'] == True:
        cog.outl("<TABHERE>ros::spinOnce();")
    if component['innermodelviewer']:
        cog.outl("#ifdef USE_QTGUI")
        cog.outl("<TABHERE>if (innerModelViewer) innerModelViewer->update();")
        cog.outl("<TABHERE>osgView->frame();")
        cog.outl("#endif")
        cog.outl("}")
]]]
[[[end]]]

[[[cog
if sm is not None:
	sm_implementation = "\n"
	if sm['machine']['contents']['states'] is not "none":
		for state in sm['machine']['contents']['states']:
		    if sm['machine']['default'] and state == 'compute':
		        sm_implementation += "void SpecificWorker::sm_" + state + "()\n{\n<TABHERE>std::cout<<\"Entered state "+state+"\"<<std::endl;\n<TABHERE>compute();\n}\n\n"
		    else:
			    sm_implementation += "void SpecificWorker::sm_" + state + "()\n{\n<TABHERE>std::cout<<\"Entered state "+state+"\"<<std::endl;\n}\n\n"
	if sm['machine']['contents']['initialstate'] != "none":
		sm_implementation += "void SpecificWorker::sm_" + sm['machine']['contents']['initialstate'][0] + "()\n{\n<TABHERE>std::cout<<\"Entered initial state "+sm['machine']['contents']['initialstate'][0]+"\"<<std::endl;\n}\n\n"
	if sm['machine']['contents']['finalstate'] != "none":
		sm_implementation += "void SpecificWorker::sm_" + sm['machine']['contents']['finalstate'][0] + "()\n{\n<TABHERE>std::cout<<\"Entered final state "+sm['machine']['contents']['finalstate'][0] +"\"<<std::endl;\n}\n\n"
	if sm['substates'] != "none":
		for substates in sm['substates']:
			if substates['contents']['states'] is not "none":
				for state in substates['contents']['states']:
					sm_implementation += "void SpecificWorker::sm_" + state + "()\n{\n<TABHERE>std::cout<<\"Entered state "+state+"\"<<std::endl;\n}\n\n"
			if substates['contents']['initialstate'] != "none":
				sm_implementation += "void SpecificWorker::sm_" + substates['contents']['initialstate'] + "()\n{\n<TABHERE>std::cout<<\"Entered state "+substates['contents']['initialstate']+"\"<<std::endl;\n}\n\n"
			if substates['contents']['finalstate'] != "none":
				sm_implementation += "void SpecificWorker::sm_" + substates['contents']['finalstate'] + "()\n{\n<TABHERE>std::cout<<\"Entered state "+substates['contents']['finalstate']+"\"<<std::endl;\n}\n\n"
	cog.outl(sm_implementation)


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
								if component['language'].lower() == "cpp":
									const = 'const '
								else:
									const = ''
									ampersand = ''
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl(method['return'] + ' SpecificWorker::' +interface['name'] + "_" + method['name'] + '(' + paramStrA + ")\n{\n//implementCODE\n"+bodyCode+"\n}\n")
					else:
						paramStrA = module['name'] +"ROS::"+method['name']+"::Request &req, "+module['name']+"ROS::"+method['name']+"::Response &res"
						if imp in component['iceInterfaces']:
							cog.outl('bool SpecificWorker::ROS' + method['name'] + '(' + paramStrA + ")\n{\n//implementCODE\n"+bodyCode+"\n}\n")
						else:
							cog.outl('bool SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n//implementCODE\n"+bodyCode+"\n}\n")

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
								if component['language'].lower() == "cpp":
									const = 'const '
								else:
									const = ''
									ampersand = ''
								if p['type'].lower() in ['int', '::ice::int', 'float', '::ice::float']:
									ampersand = ''
							# STR
							paramStrA += delim + const + p['type'] + ' ' + ampersand + p['name']
						cog.outl(method['return'] + ' SpecificWorker::' +interface['name'] + "_" + method['name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n"+bodyCode+"\n}\n")
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
							cog.outl('void SpecificWorker::ROS' + method['name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n"+bodyCode+"\n}\n")
						else:
							cog.outl('void SpecificWorker::' + method['name'] + '(' + paramStrA + ")\n{\n//subscribesToCODE\n"+bodyCode+"\n}\n")
]]]
[[[end]]]

[[[cog
try:
	if ('agmagent' in [ x.lower() for x in component['options'] ]) and ('innermodelviewer' in [ x.lower() for x in component['options'] ]):
		cog.outl("""
void SpecificWorker::regenerateInnerModelViewer()
{
	if (innerModelViewer)
	{
		osgView->getRootGroup()->removeChild(innerModelViewer);
	}

	innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
}\n""")

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
	{""")
		agentName=component['name']
		if component['language'].lower() == "cpp":
			cog.outl("<TABHERE><TABHERE>AGMMisc::publishModification(newModel, agmexecutive_proxy, \""+ agentName+"Agent\");")
		else:
			cog.outl("<TABHERE><TABHERE>AGMMisc::publishModification(newModel, *agmexecutive_proxy.get(), \""+ agentName+"Agent\");")
		cog.outl ("""<TABHERE>}
/*	catch(const RoboCompAGMExecutive::Locked &e)
	{
	}
	catch(const RoboCompAGMExecutive::OldModel &e)
	{
	}
	catch(const RoboCompAGMExecutive::InvalidChange &e)
	{
	}
*/
	catch(const Ice::Exception& e)
	{
		exit(1);
	}
}""")


except:
	pass
]]]
[[[end]]]
