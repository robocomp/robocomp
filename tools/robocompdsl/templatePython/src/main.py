#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import getNameNumber
includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)

from parseIDSL import IDSLPool
pool = IDSLPool(theIDSLs, includeDirectories)

REQUIRE_STR = """
<TABHERE># Remote object connection for <NORMAL>
<TABHERE>try:
<TABHERE><TABHERE>proxyString = ic.getProperties().getProperty('<NORMAL><NUM>Proxy')
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE>basePrx = ic.stringToProxy(proxyString)
<TABHERE><TABHERE><TABHERE><LOWER><NUM>_proxy = <NORMAL>Prx.checkedCast(basePrx)
<TABHERE><TABHERE><TABHERE>mprx["<NORMAL>Proxy<NUM>"] = <LOWER><NUM>_proxy
<TABHERE><TABHERE>except Ice.Exception:
<TABHERE><TABHERE><TABHERE>print('Cannot connect to the remote object (<NORMAL>)', proxyString)
<TABHERE><TABHERE><TABHERE>#traceback.print_exc()
<TABHERE><TABHERE><TABHERE>status = 1
<TABHERE>except Ice.Exception as e:
<TABHERE><TABHERE>print(e)
<TABHERE><TABHERE>print('Cannot get <NORMAL>Proxy property.')
<TABHERE><TABHERE>status = 1
"""

SUBSCRIBESTO_STR = """
<TABHERE><NORMAL>_adapter = ic.createObjectAdapter("<NORMAL>Topic")
<TABHERE><LOWER>I_ = <NORMAL>I(worker)
<TABHERE><LOWER>_proxy = <NORMAL>_adapter.addWithUUID(<LOWER>I_).ice_oneway()

<TABHERE>subscribeDone = False
<TABHERE>while not subscribeDone:
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager.retrieve("<NORMAL>")
<TABHERE><TABHERE><TABHERE>subscribeDone = True
<TABHERE><TABHERE>except Ice.Exception as e:
<TABHERE><TABHERE><TABHERE>print("Error. Topic does not exist (creating)")
<TABHERE><TABHERE><TABHERE>time.sleep(1)
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager.create("<NORMAL>")
<TABHERE><TABHERE><TABHERE><TABHERE>subscribeDone = True
<TABHERE><TABHERE><TABHERE>except:
<TABHERE><TABHERE><TABHERE><TABHERE>print("Error. Topic could not be created. Exiting")
<TABHERE><TABHERE><TABHERE><TABHERE>status = 0
<TABHERE>qos = {}
<TABHERE><LOWER>_topic.subscribeAndGetPublisher(qos, <LOWER>_proxy)
<TABHERE><NORMAL>_adapter.activate()
"""

PUBLISHES_STR = """
<TABHERE># Create a proxy to publish a <NORMAL> topic
<TABHERE>topic = False
<TABHERE>try:
<TABHERE><TABHERE>topic = topicManager.retrieve("<NORMAL>")
<TABHERE>except:
<TABHERE><TABHERE>pass
<TABHERE>while not topic:
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE>topic = topicManager.retrieve("<NORMAL>")
<TABHERE><TABHERE>except IceStorm.NoSuchTopic:
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE>topic = topicManager.create("<NORMAL>")
<TABHERE><TABHERE><TABHERE>except:
<TABHERE><TABHERE><TABHERE><TABHERE>print('Another client created the <NORMAL> topic? ...')
<TABHERE>pub = topic.getPublisher().ice_oneway()
<TABHERE><LOWER>Topic = <NORMAL>Prx.uncheckedCast(pub)
<TABHERE>mprx["<NORMAL>Pub"] = <LOWER>Topic
"""

IMPLEMENTS_STR = """
<TABHERE>adapter = ic.createObjectAdapter('<NORMAL>')
<TABHERE>adapter.add(<NORMAL>I(worker), ic.stringToIdentity('<LOWER>'))
<TABHERE>adapter.activate()
"""
]]]
[[[end]]]

#
# Copyright (C)
[[[cog
A()
import datetime
cog.out(' '+str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

# \mainpage RoboComp::
[[[cog
A()
cog.out(component['name'])
]]]
[[[end]]]
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/
[[[cog
A()
cog.out(component['name'])
Z()
]]]
[[[end]]]
 --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, IceStorm, time, os, copy

# Ctrl+c handling
import signal

from PySide2 import QtCore
[[[cog
    if component['gui'] is not None:
        cog.outl('from PySide2 import QtWidgets')
]]]
[[[end]]]

from specificworker import *


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler):
		self.handler = _handler
	def getFreq(self, current = None):
		self.handler.getFreq()
	def setFreq(self, freq, current = None):
		self.handler.setFreq()
	def timeAwake(self, current = None):
		try:
			return self.handler.timeAwake()
		except:
			print('Problem getting timeAwake')
	def killYourSelf(self, current = None):
		self.handler.killYourSelf()
	def getAttrList(self, current = None):
		try:
			return self.handler.getAttrList()
		except:
			print('Problem getting getAttrList')
			traceback.print_exc()
			status = 1
			return

#SIGNALS handler
def sigint_handler(*args):
	QtCore.QCoreApplication.quit()
    
if __name__ == '__main__':
[[[cog
	if component['gui'] is not None:
		cog.outl('<TABHERE>app = QtWidgets.QApplication(sys.argv)')
	else:
		cog.outl('<TABHERE>app = QtCore.QCoreApplication(sys.argv)')
]]]
[[[end]]]
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	elif len(params) == 1:
		params.append('--Ice.Config=config')
	ic = Ice.initialize(params)
	status = 0
	mprx = {}
	parameters = {}
	for i in ic.getProperties():
		parameters[str(i)] = str(ic.getProperties().getProperty(i))
[[[cog

try:
	needIce = False
	needStorm = False
	for req in component['requires']:
		if communicationIsIce(req):
			needIce = True
	for imp in component['implements']:
		if communicationIsIce(imp):
			needIce = True
	for pub in component['publishes']:
		if communicationIsIce(pub):
			needIce = True
			needStorm = True
	for sub in component['subscribesTo']:
		if communicationIsIce(sub):
			needIce = True
			needStorm = True
	if needStorm:
		cog.outl("""
<TABHERE># Topic Manager
<TABHERE>proxy = ic.getProperties().getProperty("TopicManager.Proxy")
<TABHERE>obj = ic.stringToProxy(proxy)
<TABHERE>try:
<TABHERE><TABHERE>topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
<TABHERE>except Ice.ConnectionRefusedException as e:
<TABHERE><TABHERE>print('Cannot connect to IceStorm! ('+proxy+')')
<TABHERE><TABHERE>status = 1""")
except:
	pass

for req, num in getNameNumber(component['requires']):
	if communicationIsIce(req):
		w = REQUIRE_STR.replace("<NORMAL>", req).replace("<LOWER>", req.lower()).replace("<NUM>",num)
		cog.outl(w)

for pub, num in getNameNumber(component['publishes']):
	if communicationIsIce(pub):
		w = PUBLISHES_STR.replace("<NORMAL>", pub).replace("<LOWER>", pub.lower())
		cog.outl(w)

cog.outl("<TABHERE>if status == 0:")
cog.outl("<TABHERE><TABHERE>worker = SpecificWorker(mprx)")
cog.outl("<TABHERE><TABHERE>worker.setParams(parameters)")
cog.outl("<TABHERE>else:")
cog.outl("<TABHERE><TABHERE>print(\"Error getting required connections, check config file\")")
cog.outl("<TABHERE><TABHERE>sys.exit(-1)")

for imp in component['implements']:
	if communicationIsIce(imp):
		w = IMPLEMENTS_STR.replace("<NORMAL>", imp).replace("<LOWER>", imp.lower())
		cog.outl(w)

for sut in component['subscribesTo']:
	if communicationIsIce(sut):
		w = SUBSCRIBESTO_STR.replace("<NORMAL>", sut).replace("<LOWER>", sut.lower())
		cog.outl(w)
if component['usingROS'] == True:
	cog.outl("<TABHERE>rospy.init_node(\""+component['name']+"\", anonymous=True)")
for sub in component['subscribesTo']:
	nname = sub
	while type(nname) != type(''):
		nname = nname[0]
	module = pool.moduleProviding(nname)
	if module == None:
		print('\nCan\'t find module providing', nname, '\n')
		sys.exit(-1)
	if not communicationIsIce(sub):
		for interface in module['interfaces']:
			if interface['name'] == nname:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					for p in method['params']:
						s = "\""+mname+"\""
						if p['type'] in ('float','int'):
							cog.outl("<TABHERE>rospy.Subscriber("+s+", "+p['type'].capitalize()+"32, worker.ROS"+method['name']+")")
						elif p['type'] in ('uint8','uint16','uint32','uint64'):
							cog.outl("<TABHERE>rospy.Subscriber("+s+", UInt"+p['type'].split('t')[1]+", worker.ROS"+method['name']+")")
						elif p['type'] in rosTypes:
							cog.outl("<TABHERE>rospy.Subscriber("+s+", "+p['type'].capitalize()+", worker.ROS"+method['name']+")")
						elif '::' in p['type']:
							cog.outl("<TABHERE>rospy.Subscriber("+s+", "+p['type'].split('::')[1]+", worker.ROS"+method['name']+")")
						else:
							cog.outl("<TABHERE>rospy.Subscriber("+s+", "+p['type']+", worker.ROS"+method['name']+")")

for imp in component['implements']:
	nname = imp
	while type(nname) != type(''):
		nname = nname[0]
	module = pool.moduleProviding(nname)
	if module == None:
		print('\nCan\'t find module providing', nname, '\n')
		sys.exit(-1)
	if not communicationIsIce(imp):
		for interface in module['interfaces']:
			if interface['name'] == nname:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					s = "\""+mname+"\""
					cog.outl("<TABHERE>rospy.Service("+s+", "+mname+", worker.ROS"+method['name']+")")

]]]
[[[end]]]

	signal.signal(signal.SIGINT, sigint_handler)
	app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
