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
if component['useViewer'] == "true":
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>imv = NULL;")
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
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
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
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("""//	THE FOLLOWING IS JUST AN EXAMPLE for AGENTS
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("NameAgent.InnerModel") ;
//		if( QFile(QString::fromStdString(par.value)).exists() == true)
//		{
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Reading Innermodel file " << QString::fromStdString(par.value);
//			innerModel = new InnerModel(par.value);
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file read OK!" ;
//		}
//		else
//		{
//			qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "Innermodel file " << QString::fromStdString(par.value) << " does not exists";
//			qFatal("Exiting now.");
//		}
//	}
//	catch(std::exception e)
//	{
//		qFatal("Error reading config params");
//	}""")
except:
	cog.outl("""//       THE FOLLOWING IS JUST AN EXAMPLE for components
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path=par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }""")
if component['useViewer'] == "true":
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>imv = new InnerModelViewer (innerModel, \"root\", osgView->getRootGroup(), true);")
	cog.outl("#endif")
]]]
[[[end]]]	

	
	timer.start(Period);
	
[[[cog
try:
	if 'agmagent' in [ x.lower() for x in component['options'] ]:
		cog.outl("<TABHERE>try")
		cog.outl("<TABHERE>{")
		cog.outl("<TABHERE><TABHERE>RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();")
		cog.outl("<TABHERE><TABHERE>structuralChange(w);")
		cog.outl("<TABHERE>}")
		cog.outl("<TABHERE>catch(...)")
		cog.outl("<TABHERE>{")
		cog.outl("<TABHERE><TABHERE>printf(\"The executive is probably not running, waiting for first AGM model publication...\");")
		cog.outl("<TABHERE>}")

except:
	pass
]]]
[[[end]]]

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
[[[cog
if component['usingROS'] == True:
	cog.outl("<TABHERE>ros::spinOnce();")
if component['useViewer'] == "true":
	cog.outl("#ifdef USE_QTGUI")
	cog.outl("<TABHERE>if (imv) imv->update();")
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
					bodyCode = bodyCodeFromName(method['name'])
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
						paramStrA = module['name'] +"::"+method['name']+"::Request &req, "+module['name']+"::"+method['name']+"::Response &res"
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
					bodyCode = bodyCodeFromName(method['name'])
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
							elif p['type'] == 'string':
								p['type'] = "std_msgs::String"
							elif not '::' in p['type']:
								p['type'] = module['name']+"::"+p['type']
							# STR
							paramStrA += delim + p['type'] + ' ' + p['name']
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




