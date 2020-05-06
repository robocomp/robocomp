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

import templateCPP.functions.src.genericworker_cpp as genericworker
from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice, IDSLPool
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component.statemachine_path)
if sm is None:
    component.statemachine_path = None
if component is None:
    raise ValueError('genericworker.cpp: Can\'t locate %s' % theCDSL)

pool = IDSLPool(theIDSLs, includeDirectories)


]]]
[[[end]]]
 [[[cog
 import datetime
 cog.out("*    Copyright (C)"+str(datetime.date.today().year)+" by YOUR NAME HERE")
 ]]]
 [[[end]]]
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
[[[cog
if component.language.lower() == 'cpp':
	cog.outl("GenericWorker::GenericWorker(MapPrx& mprx) :")
else:
	cog.outl("GenericWorker::GenericWorker(TuplePrx tprx) :")
if component.gui is not None:
	cog.outl("""#ifdef USE_QTGUI
Ui_guiDlg()
#else
QObject()
#endif
""")
else:
	cog.outl("QObject()")
]]]
[[[end]]]
{

	[[[cog
	cog.out(genericworker.statemachine_initialization(sm, component.statemachine_visual))

	cog.out(genericworker.require_and_publish_proxies_creation(component))
	]]]
	[[[end]]]

	mutex = new QMutex(QMutex::Recursive);

	[[[cog
	cog.out(genericworker.ros_nodes_creation(component, pool))
	cog.out(genericworker.ros_proxies_creation(component))
	cog.out(genericworker.gui_setup(component.gui))
	]]]
	[[[end]]]
	Period = BASIC_PERIOD;
	[[[cog
	if component.statemachine_path is None:
		cog.outl("connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));")
	]]]
	[[[end]]]

}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}
//[[[cog
//cog.out(genericworker.agm_methods(component))
//]]]
//[[[end]]]
