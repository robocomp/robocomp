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

from cogapp.whiteutils import reindentBlock
import templateCPP.functions.src.specificworker_cpp as specificworker
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, is_agm1_agent, is_agm2_agent, is_agm2_agent_ROS, IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component.statemachine)
if sm is None:
    component.statemachine = None
if component is None:
    raise ValueError('specificworker.cpp: Can\'t locate %s' % theCDSL)



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
#include "specificworker.h"

/**
* \brief Default constructor
*/
[[[cog
    if component.language.lower() == 'cpp':
        cog.outl("SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)")
    else:
        cog.outl("SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)")
]]]
[[[end]]]
{

[[[cog
cog.out(specificworker.innermodelviewer_code(component.innermodelviewer))
cog.out(specificworker.agmagent_attributes(component))

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
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }


[[[cog
cog.out(specificworker.innermodel_and_viewer_attribute_init(component.innermodelviewer))
]]]
[[[end]]]

[[[cog
cog.out(specificworker.agm_innermodel_association(component))
]]]
[[[end]]]

[[[cog
if sm is not None:
    cog.outl("<TABHERE>" + sm['machine']['name'] + ".start();")
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
cog.out(specificworker.compute_method(component, sm))
]]]
[[[end]]]

[[[cog
cog.out(specificworker.statemachine_methods_creation(sm))
]]]
[[[end]]]



[[[cog
cog.out(specificworker.implements(component, pool))
cog.out(specificworker.subscribes(component, pool))
]]]
[[[end]]]

[[[cog
cog.out(reindentBlock(specificworker.agm_specific_code(component)))
]]]
[[[end]]]
