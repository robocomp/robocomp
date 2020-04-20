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

import templateCPP.functions.src.specificworker_h as functions
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent, IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component.statemachine)
if sm is None:
    component.statemachine = None
if component is None:
	raise ValueError('specificworker.h: Can\'t locate %s' % theCDSLs)

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
	if 'agmagent' in [ x.lower() for x in component.options ]:
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
if component.innermodelviewer:
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
	if component.language.lower() == 'cpp':
		cog.outl("<TABHERE>SpecificWorker(MapPrx& mprx);")
	else:
		cog.outl("<TABHERE>SpecificWorker(TuplePrx tprx);")
]]]
[[[end]]]
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

[[[cog
cog.out(functions.specificworker_implements_method_definitions(pool, component))

cog.out(functions.specificworker_subscribes_method_definitions(pool, component))

]]]
[[[end]]]

public slots:
[[[cog
if (sm is not None and sm['machine']['default'] is True) or component.statemachine is None:
	cog.outl("<TABHERE>void compute();")
]]]
[[[end]]]
	void initialize(int period);
[[[cog
cog.out(functions.specificworker_statemachine_methods_definitions(component, sm))
]]]
[[[end]]]
private:
	std::shared_ptr<InnerModel> innerModel;
[[[cog
cog.out(functions.specificworker_innermodelviewer_attributes(component.innermodelviewer))
cog.out(functions.specificworker_agm_attributes(component))

]]]
[[[end]]]

};

#endif
