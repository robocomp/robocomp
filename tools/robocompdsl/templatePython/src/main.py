#!/usr/bin/env python
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

from parseCDSL import *
component = CDSLParsing.fromFile(theCDSL)

from parseIDSL import *
pool = IDSLPool(theIDSLs)

REQUIRE_STR = """
<TABHERE><TABHERE># Remote object connection for <NORMAL>
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE>proxyString = ic.getProperties().getProperty('<NORMAL><NUM>Proxy')
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE>basePrx = ic.stringToProxy(proxyString)
<TABHERE><TABHERE><TABHERE><TABHERE><LOWER><NUM>_proxy = <NORMAL>Prx.checkedCast(basePrx)
<TABHERE><TABHERE><TABHERE><TABHERE>mprx["<NORMAL>Proxy<NUM>"] = <LOWER><NUM>_proxy
<TABHERE><TABHERE><TABHERE>except Ice.Exception:
<TABHERE><TABHERE><TABHERE><TABHERE>print 'Cannot connect to the remote object (<NORMAL>)', proxyString
<TABHERE><TABHERE><TABHERE><TABHERE>#traceback.print_exc()
<TABHERE><TABHERE><TABHERE><TABHERE>status = 1
<TABHERE><TABHERE>except Ice.Exception, e:
<TABHERE><TABHERE><TABHERE>print e
<TABHERE><TABHERE><TABHERE>print 'Cannot get <NORMAL>Proxy property.'
<TABHERE><TABHERE><TABHERE>status = 1
"""

SUBSCRIBESTO_STR = """
<TABHERE><TABHERE><NORMAL>_adapter = ic.createObjectAdapter("<NORMAL>Topic")
<TABHERE><TABHERE><LOWER>I_ = <NORMAL>I(worker)
<TABHERE><TABHERE><LOWER>_proxy = <NORMAL>_adapter.addWithUUID(<LOWER>I_).ice_oneway()

<TABHERE><TABHERE>subscribeDone = False
<TABHERE><TABHERE>while not subscribeDone:
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE><LOWER>_topic = topicManager.retrieve("<NORMAL>")
<TABHERE><TABHERE><TABHERE><TABHERE>subscribeDone = True
<TABHERE><TABHERE><TABHERE>except Ice.Exception, e:
<TABHERE><TABHERE><TABHERE><TABHERE>print "Error. Topic does not exist (yet)"
<TABHERE><TABHERE><TABHERE><TABHERE>status = 0
<TABHERE><TABHERE><TABHERE><TABHERE>time.sleep(1)
<TABHERE><TABHERE>qos = {}
<TABHERE><TABHERE><LOWER>_topic.subscribeAndGetPublisher(qos, <LOWER>_proxy)
<TABHERE><TABHERE><NORMAL>_adapter.activate()
"""

PUBLISHES_STR = """
<TABHERE><TABHERE># Create a proxy to publish a <NORMAL> topic
<TABHERE><TABHERE>topic = False
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE>topic = topicManager.retrieve("<NORMAL>")
<TABHERE><TABHERE>except:
<TABHERE><TABHERE><TABHERE>pass
<TABHERE><TABHERE>while not topic:
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE>topic = topicManager.retrieve("<NORMAL>")
<TABHERE><TABHERE><TABHERE>except IceStorm.NoSuchTopic:
<TABHERE><TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>topic = topicManager.create("<NORMAL>")
<TABHERE><TABHERE><TABHERE><TABHERE>except:
<TABHERE><TABHERE><TABHERE><TABHERE><TABHERE>print 'Another client created the <NORMAL> topic? ...'
<TABHERE><TABHERE>pub = topic.getPublisher().ice_oneway()
<TABHERE><TABHERE><LOWER>Topic = <NORMAL>Prx.uncheckedCast(pub)
<TABHERE><TABHERE>mprx["<NORMAL>Pub"] = <LOWER>Topic
"""

IMPLEMENTS_STR = """
<TABHERE><TABHERE>adapter = ic.createObjectAdapter('<NORMAL>')
<TABHERE><TABHERE>adapter.add(<NORMAL>I(worker), ic.stringToIdentity('<LOWER>'))
<TABHERE><TABHERE>adapter.activate()
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

import sys, traceback, IceStorm, subprocess, threading, time, Queue, os, copy

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *

from specificworker import *


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler, _communicator):
		self.handler = _handler
		self.communicator = _communicator
	def getFreq(self, current = None):
		self.handler.getFreq()
	def setFreq(self, freq, current = None):
		self.handler.setFreq()
	def timeAwake(self, current = None):
		try:
			return self.handler.timeAwake()
		except:
			print 'Problem getting timeAwake'
	def killYourSelf(self, current = None):
		self.handler.killYourSelf()
	def getAttrList(self, current = None):
		try:
			return self.handler.getAttrList(self.communicator)
		except:
			print 'Problem getting getAttrList'
			traceback.print_exc()
			status = 1
			return



if __name__ == '__main__':
[[[cog
	if component['gui'] != "none":
		cog.outl('<TABHERE>app = QtGui.QApplication(sys.argv)')
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
	for req in component['requires']:
		if communicationIsIce(req):
			needIce = True
	for imp in component['implements']:
		if communicationIsIce(imp):
			needIce = True
	for pub in component['publishes']:
		if communicationIsIce(pub):
			needIce = True
	for sub in component['subscribesTo']:
		if communicationIsIce(sub):
			needIce = True
	if needIce:
		cog.outl("""
<TABHERE><TABHERE># Topic Manager
<TABHERE><TABHERE>proxy = ic.getProperties().getProperty("TopicManager.Proxy")
<TABHERE><TABHERE>obj = ic.stringToProxy(proxy)
<TABHERE><TABHERE>topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)""")
except:
	pass

for req, num in getNameNumber(component['requires']):
	if type(req) == str:
		rq = req
	else:
		rq = req[0]
	if communicationIsIce(req):
		w = REQUIRE_STR.replace("<NORMAL>", rq).replace("<LOWER>", rq.lower()).replace("<NUM>",num)
		cog.outl(w)

for pb, num in getNameNumber(component['publishes']):
	if type(pb) == str:
		pub = pb
	else:
		pub = pb[0]
	if communicationIsIce(pb):
		w = PUBLISHES_STR.replace("<NORMAL>", pub).replace("<LOWER>", pub.lower())
		cog.outl(w)

cog.outl("<TABHERE>if status == 0:")
cog.outl("<TABHERE><TABHERE>worker = SpecificWorker(mprx)")
cog.outl("<TABHERE><TABHERE>worker.setParams(parameters)")
for im in component['implements']:
	if type(im) == str:
		imp = im
	else:
		imp = im[0]
	if communicationIsIce(im):
		w = IMPLEMENTS_STR.replace("<NORMAL>", imp).replace("<LOWER>", imp.lower())
		cog.outl(w)

for sut in component['subscribesTo']:
	if type(sut) == str:
		st = sut
	else:
		st = sut[0]
	if communicationIsIce(sut):
		w = SUBSCRIBESTO_STR.replace("<NORMAL>", st).replace("<LOWER>", st.lower())
		cog.outl(w)
if component['usingROS'] == True:
	cog.outl("<TABHERE><TABHERE>rospy.init_node(\""+component['name']+"\", anonymous=True)")
for sub in component['subscribesTo']:
	nname = sub
	while type(nname) != type(''):
		nname = nname[0]
	module = pool.moduleProviding(nname)
	if module == None:
		print ('\nCan\'t find module providing', nname, '\n')
		sys.exit(-1)
	if not communicationIsIce(sub):
		for interface in module['interfaces']:
			if interface['name'] == nname:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					for p in method['params']:
						s = "\""+mname+"\""
						if p['type'] in ('float','int','uint'):
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type'].capitalize()+"32, worker.ROS"+method['name']+")")
						elif p['type'] == 'string':
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", String, worker.ROS"+method['name']+")")
						elif '::' in p['type']:
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type'].split('::')[1]+", worker.ROS"+method['name']+")")
						else:
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type']+", worker.ROS"+method['name']+")")

for imp in component['implements']:
	nname = imp
	while type(nname) != type(''):
		nname = nname[0]
	module = pool.moduleProviding(nname)
	if module == None:
		print ('\nCan\'t find module providing', nname, '\n')
		sys.exit(-1)
	if not communicationIsIce(imp):
		for interface in module['interfaces']:
			if interface['name'] == nname:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					s = "\""+mname+"\""
					cog.outl("<TABHERE><TABHERE>rospy.Service("+s+", "+mname+", worker.ROS"+method['name']+")")

]]]
[[[end]]]

		app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
