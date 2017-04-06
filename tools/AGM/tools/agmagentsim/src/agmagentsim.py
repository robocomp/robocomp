#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2016 by YOUR NAME HERE
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

# \mainpage RoboComp::agmagentsim
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
# Just: "${PATH_TO_BINARY}/agmagentsim --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, IceStorm, subprocess, threading, time, Queue, os, copy

sys.path.append('/usr/local/bin/agmagentsim_py/')

# Ctrl+c handling
import signal

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
	app = QtGui.QApplication(sys.argv)
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

	# Topic Manager
	proxy = ic.getProperties().getProperty("TopicManager.Proxy")
	obj = ic.stringToProxy(proxy)
	try:
		topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
	except Ice.ConnectionRefusedException, e:
		print 'Cannot connect to IceStorm! ('+proxy+')'
		sys.exit(-1)

	# Remote object connection for AGMExecutive
	try:
		proxyString = ic.getProperties().getProperty('AGMExecutiveProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			agmexecutive_proxy = AGMExecutivePrx.checkedCast(basePrx)
			mprx["AGMExecutiveProxy"] = agmexecutive_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (AGMExecutive)', proxyString
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get AGMExecutiveProxy property.'
		status = 1

	if status != 0:
		print 'Fatal error'
		
	
	worker = SpecificWorker(mprx)
	worker.setParams(parameters)

	adapter = ic.createObjectAdapter('AGMCommonBehavior')
	adapter.add(AGMCommonBehaviorI(worker), ic.stringToIdentity('agmcommonbehavior'))
	adapter.activate()


	AGMExecutiveTopic_adapter = ic.createObjectAdapter("AGMExecutiveTopicTopic")
	agmexecutivetopicI_ = AGMExecutiveTopicI(worker)
	agmexecutivetopic_proxy = AGMExecutiveTopic_adapter.addWithUUID(agmexecutivetopicI_).ice_oneway()

	subscribeDone = False
	while not subscribeDone:
		try:
			agmexecutivetopic_topic = topicManager.retrieve("AGMExecutiveTopic")
			subscribeDone = True
		except Ice.Exception, e:
			print "Error. Topic does not exist (yet)"
			status = 0
			time.sleep(1)
	qos = {}
	agmexecutivetopic_topic.subscribeAndGetPublisher(qos, agmexecutivetopic_proxy)
	AGMExecutiveTopic_adapter.activate()


	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
