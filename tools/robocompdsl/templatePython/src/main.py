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
<TABHERE># Remote object connection for <NORMAL><NUM>
<TABHERE>proxyData["<NORMAL><NUM>"] = {"comp":"@@COMP NAME HERE@@","caster":<NORMAL>Prx.checkedCast,"name":"<NORMAL>"}
<TABHERE>try:
<TABHERE><TABHERE>while True:
<TABHERE><TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE><TABHERE>interfaces = rcmaster_proxy.getComp(proxyData["<NORMAL><NUM>"]["comp"],"localhost");
<TABHERE><TABHERE><TABHERE><TABHERE>iface = [x for x in interfaces if x.name == proxyData["<NORMAL><NUM>"]["name"]][0]
<TABHERE><TABHERE><TABHERE><TABHERE>basePrx = ic.stringToProxy(proxyData["<NORMAL><NUM>"]["name"]+":"+iface.protocol+" -h localhost -p "+str(iface.port))
<TABHERE><TABHERE><TABHERE><TABHERE>proxyData["<NORMAL><NUM>"]["proxy"] = proxyData["<NORMAL><NUM>"]["caster"](basePrx)
<TABHERE><TABHERE><TABHERE>except ComponentNotFound:
<TABHERE><TABHERE><TABHERE><TABHERE>print 'waiting for <NORMAL><NUM> interface'
<TABHERE><TABHERE><TABHERE>except IndexError:
<TABHERE><TABHERE><TABHERE><TABHERE>raise Exception(proxyData["<NORMAL><NUM>"]["comp"]+" dosnt provide"+proxyData["<NORMAL><NUM>"]["name"])
<TABHERE><TABHERE><TABHERE><TABHERE>time.sleep(3)
<TABHERE><TABHERE><TABHERE>except Ice.Exception:
<TABHERE><TABHERE><TABHERE><TABHERE>print 'Cannot connect to the remote object (<NORMAL>)'
<TABHERE><TABHERE><TABHERE><TABHERE>traceback.print_exc()
<TABHERE><TABHERE><TABHERE><TABHERE>status = 1
<TABHERE><TABHERE><TABHERE>else:
<TABHERE><TABHERE><TABHERE><TABHERE>break
<TABHERE>except Ice.Exception, e:
<TABHERE><TABHERE>print e
<TABHERE><TABHERE>print 'Cannot get <NORMAL>Proxy property.'
<TABHERE><TABHERE>status = 1
"""

REQUIRE_STR_RCMASTER="""
<TABHERE># Remote object connection for rcmaster
<TABHERE>proxyData["rcmaster"] = {"comp":"rcmaster","caster":rcmasterPrx.checkedCast,"name":"rcmaster"}
<TABHERE>try:
<TABHERE><TABHERE>with open(os.path.join(os.path.expanduser('~'), ".config/RoboComp/rcmaster.config"), 'r') as f:
<TABHERE><TABHERE><TABHERE>rcmaster_uri = f.readline().strip().split(":")
<TABHERE><TABHERE>basePrx = ic.stringToProxy("rcmaster:tcp -h "+rcmaster_uri[0]+" -p "+rcmaster_uri[1])
<TABHERE><TABHERE>try:
<TABHERE><TABHERE><TABHERE>print "Connecting to rcmaster " ,rcmaster_uri
<TABHERE><TABHERE><TABHERE>rcmaster_proxy = rcmasterPrx.checkedCast(basePrx)
<TABHERE><TABHERE>except Ice.SocketException:
<TABHERE><TABHERE><TABHERE>raise Exception("RCMaster is not running")
<TABHERE><TABHERE>proxyData["rcmaster"]["proxy"] = rcmaster_proxy
<TABHERE>except Ice.Exception:
<TABHERE><TABHERE>print 'Cannot connect to the remote object (rcmaster)'
<TABHERE><TABHERE>traceback.print_exc()
<TABHERE><TABHERE>status = 1
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

IMPLEMENTS_STR = """<TABHERE><TABHERE>compInfo.interfaces.append(interfaceData("<NORMAL>"))"""

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
	mprx["name"] = ic.getProperties().getProperty('Ice.ProgramName')
	proxyData = {}
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
<TABHERE># Topic Manager
<TABHERE>proxy = ic.getProperties().getProperty("TopicManager.Proxy")
<TABHERE>obj = ic.stringToProxy(proxy)
<TABHERE>topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)""")
except:
	pass

for req, num in getNameNumber(component['requires']):
	if num == '1':
		num = ''
	if type(req) == str:
		rq = req
	else:
		rq = req[0]
	if communicationIsIce(req):
		if rq.lower() == "rcmaster":
			w = REQUIRE_STR_RCMASTER
		else:
			w = REQUIRE_STR.replace("<NORMAL>", rq).replace("<NUM>", num).replace("<LOWER>", rq.lower())
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
cog.outl("<TABHERE><TABHERE>mprx['proxyData'] = proxyData")
cog.outl("<TABHERE><TABHERE>worker = SpecificWorker(mprx)")
cog.outl("<TABHERE><TABHERE>worker.setParams(parameters)")

if len(component['implements']) > 0:
	cog.outl("""<TABHERE><TABHERE>worker = SpecificWorker(mprx)
<TABHERE><TABHERE>compInfo = compData(name=mprx["name"])
<TABHERE><TABHERE>compInfo.interfaces = []""")
	for ima in component['implements']:
		if type(ima) == type(''):
			im = ima
		else:
			im = ima[0]
		w = IMPLEMENTS_STR.replace("<NORMAL>", im).replace("<LOWER>", im.lower())
		cog.outl(w)

	cog.outl("""<TABHERE><TABHERE>idata = rcmaster_proxy.registerComp(compInfo,False,True)
		
<TABHERE><TABHERE># activate all interfaces
<TABHERE><TABHERE>for iface in idata:
<TABHERE><TABHERE><TABHERE>adapter = ic.createObjectAdapterWithEndpoints(iface.name.lower(), iface.protocol+' -h localhost -p '+str(iface.port))
<TABHERE><TABHERE><TABHERE>workerObj = globals()[str(iface.name)+'I'](worker)
<TABHERE><TABHERE><TABHERE>adapter.add(workerObj, ic.stringToIdentity(str(iface.name).lower()))
<TABHERE><TABHERE><TABHERE>adapter.activate()
<TABHERE><TABHERE><TABHERE>print "activated interface :", iface
<TABHERE><TABHERE>print "Component Started"
""")

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
						s = "\""+nname+"_"+mname+"\""
						if p['type'] in ('float','int','uint'):
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type'].capitalize()+"32, worker."+method['name']+")")
						elif p['type'] == 'string':
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", String, worker."+method['name']+")")
						elif '::' in p['type']:
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type'].split('::')[1]+", worker."+method['name']+")")
						else:
							cog.outl("<TABHERE><TABHERE>rospy.Subscriber("+s+", "+p['type']+", worker."+method['name']+")")

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
					s = "\""+nname+"_"+mname+"\""
					cog.outl("<TABHERE><TABHERE>rospy.Service("+s+", "+mname+", worker."+method['name']+")")

]]]
[[[end]]]

		app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
