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

import templateCPP.functions.src.genericworker_h as genericworker
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice, IDSLPool, is_agm1_agent,is_agm2_agent
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component.statemachine)
if sm is None:
    component.statemachine = None
if component is None:
    raise ValueError('genericworker.h: Can\'t locate %s' % theCDSL)

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
cog.out(genericworker.gui_includes(component.gui))
]]]
[[[end]]]
[[[cog
cog.out(genericworker.statemachine_includes(sm, component.statemachine_visual))
]]]
[[[end]]]
#include <CommonBehavior.h>

[[[cog
cog.out(genericworker.interfaces_includes(component, pool))
cog.out(genericworker.agm_includes(component))
]]]
[[[end]]]


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

using namespace std;
[[[cog
cog.out(genericworker.namespaces(component))
]]]
[[[end]]]

[[[cog
cog.out(genericworker.ice_proxies_map(component))
]]]
[[[end]]]

[[[cog
cog.out(genericworker.agm_behaviour_parameter_struct(component))
]]]
[[[end]]]

[[[cog
if component.usingROS == True:
	genericworker.ros_publishes_classes(component, pool)

	genericworker.ros_requires_classes(component, pool)
]]]
[[[end]]]
class GenericWorker :
[[[cog
if component.gui is not None:
	cog.outl("#ifdef USE_QTGUI\n<TABHERE>public " + component.gui[1] + ", public Ui_guiDlg\n#else\n<TABHERE>public QObject\n #endif")
else:
	cog.outl("public QObject")
]]]
[[[end]]]
{
Q_OBJECT
public:
[[[cog
if component.language.lower() == 'cpp':
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
	cog.out(genericworker.agm_methods(component))
	]]]
	[[[end]]]


	[[[cog
	cog.out(genericworker.create_proxies(component))
	]]]
	[[[end]]]

	[[[cog
	cog.out(genericworker.implements(component, pool))
	cog.out(genericworker.subscribes(component, pool))
	]]]
	[[[end]]]

protected:
	[[[cog
	cog.out(genericworker.statemachine_creation(sm, component.statemachine_visual))
	]]]
	[[[end]]]

	QTimer timer;
	int Period;
    [[[cog
    if component.usingROS == True:
        cog.outl("ros::NodeHandle node;")
        cog.out(genericworker.ros_subscribers_creation(component, pool))
        cog.out(genericworker.ros_implements_creation(component, pool))
        cog.out(genericworker.ros_publishes_creation(component))
        cog.out(genericworker.ros_requires_creation(component))
    cog.out(genericworker.agm_attributes_creation(component))
]]]
[[[end]]]

private:


public slots:
	[[[cog
	cog.out(genericworker.statemachine_slots(sm))

	if (sm is not None and sm['machine']['default'] is True) or component.statemachine is None:
		cog.outl("virtual void compute() = 0;")
	]]]
	[[[end]]]
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
	[[[cog
	cog.out(genericworker.statemachine_signals(sm))
	]]]
	[[[end]]]
};

#endif
