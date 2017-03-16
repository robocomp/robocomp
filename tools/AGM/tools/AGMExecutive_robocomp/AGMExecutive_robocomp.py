#!/usr/bin/env python
# -*- coding: utf-8 -*-

#    Copyright (C) 2014 by Luis J. Manso
#
#    This file is part of AGM
#
#    AGM is free software: you can redistribute it and/or modify it
#    under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    AGM is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
#    the GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with AGM.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, traceback, Ice, subprocess, threading, time, Queue, os
import IceStorm



# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)
# AGM
sys.path.append('/usr/local/share/agm')

import xmlModelParser
import AGMModelConversion

# Check that RoboComp has been correctly detected
ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"AGMCommonBehavior.ice")
Ice.loadSlice(preStr+"AGMExecutive.ice")
Ice.loadSlice(preStr+"AGMWorldModel.ice")
import RoboCompAGMCommonBehavior
import RoboCompAGMExecutive
import RoboCompAGMWorldModel

from AGGL import *
from agglplanningcache import *

class ExecutiveI (RoboCompAGMExecutive.AGMExecutive):
	def __init__(self, _handler):
		self.handler = _handler

	def activate(self, current=None):
		self.handler.activate()

	def deactivate(self, current=None):
		self.handler.deactivate()

	def structuralChangeProposal(self, model, sender, log, current=None):
		self.handler.structuralChangeProposal(model, sender, log);

	def symbolUpdate(self, s, current=None):
		self.handler.symbolUpdate(s)

	def symbolsUpdate(self, s, current=None):
		self.handler.symbolsUpdate(ss)

	def edgeUpdate(self, e, current=None):
		#print 'robocomp edgesUpdate a'
		self.handler.edgeUpdate(e)
		#print 'robocomp edgesUpdate z'

	def edgesUpdate(self, es, current=None):
		self.handler.edgesUpdate(es)

	def setMission(self, target, current=None):
		self.handler.setMission(target, avoidUpdate=False)
 
	def getModel(self, current=None):
		return self.handler.getModel()
 
	def getNode(self, current=None):
		return self.handler.getNode()
 
	def getEdge(self, current=None):
		return self.handler.getEdge()

	def getData(self, current=None):
		return self.handler.getData()

	def broadcastModel(self, current=None):
		self.handler.broadcastModel()

	def broadcastPlan(self, current=None):
		self.handler.broadcastPlan()




from AGMExecutive_core import Executive


class Server (Ice.Application):
	def run (self, argv):
		status = 0
		try:
			self.shutdownOnInterrupt()

			# Get component's parameters from config file
			initialModelPath   = self.communicator().getProperties().getProperty( "InitialModelPath" )
			agglPath           = self.communicator().getProperties().getProperty( "AGGLPath" )
			initialMissionPath = self.communicator().getProperties().getProperty( "InitialMissionPath" )
			doNotPlan          = self.communicator().getProperties().getProperty( "DoNotPlan" )

			# Proxy to publish AGMExecutiveTopic
			proxy = self.communicator().getProperties().getProperty("IceStormProxy")
			obj = self.communicator().stringToProxy(proxy)
			topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
			try:
				topic = False
				topic = topicManager.retrieve("AGMExecutiveTopic")
			except:
				pass
			while not topic:
				try:
					topic = topicManager.retrieve("AGMExecutiveTopic")
				except IceStorm.NoSuchTopic:
					try:
						topic = topicManager.create("AGMExecutiveTopic")
					except:
						print 'Another client created the AGMExecutiveTopic topic... ok'
			pub = topic.getPublisher().ice_oneway()
			executiveTopic = RoboCompAGMExecutive.AGMExecutiveTopicPrx.uncheckedCast(pub)


			# Proxy to publish AGMExecutiveVisualizationTopic
			proxy = self.communicator().getProperties().getProperty("IceStormProxy");
			obj = self.communicator().stringToProxy(proxy)
			topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
			try:
				topic = False
				topic = topicManager.retrieve("AGMExecutiveVisualizationTopic")
			except:
				pass
			while not topic:
				try:
					topic = topicManager.retrieve("AGMExecutiveVisualizationTopic")
				except IceStorm.NoSuchTopic:
					try:
						topic = topicManager.create("AGMExecutiveVisualizationTopic")
					except:
						print 'Another client created the AGMExecutiveVisualizationTopic topic... ok'
			pub = topic.getPublisher().ice_oneway()
			executiveVisualizationTopic = RoboCompAGMExecutive.AGMExecutiveVisualizationTopicPrx.uncheckedCast(pub)

			# Create the executive
			executive = Executive(agglPath, initialModelPath, initialMissionPath, doNotPlan, executiveTopic, executiveVisualizationTopic)
			# AGMExecutive server
			executiveI = ExecutiveI(executive)
			adapterExecutive = self.communicator().createObjectAdapter('AGMExecutive')
			adapterExecutive.add(executiveI, self.communicator().stringToIdentity('agmexecutive'))
			adapterExecutive.activate()


			# Read agent's configurations and create the correspoding proxies
			agentConfigs = self.communicator().getProperties().getProperty( "AGENTS" ).split(',')
			print 'AGENT configs:', agentConfigs
			for agent in agentConfigs:
				print 'Configuring ', agent
				proxy = self.communicator().getProperties().getProperty(agent)
				if len(proxy)>0:
					behavior_proxy = RoboCompAGMCommonBehavior.AGMCommonBehaviorPrx.uncheckedCast(self.communicator().stringToProxy(proxy))
					if not behavior_proxy:
						print agentConfigs
						print parameters.agents[i].c_str()
						print parameters.agents[i].c_str()
						print "Error loading behavior proxy!"
						sys.exit(1)
					executive.setAgent(agent,  behavior_proxy)
					print "Agent" , agent, "initialized ok"
				else:
					print 'Agent', agent, 'was not properly configured. Check config file'

			print 'AGMExecutive initialization ok'

			print '-------------------------------------------------------------'
			print '----     R u n     A G M E x e c u t i v e,     r u n   -----'
			print '-------------------------------------------------------------'
			print '---- updatePlan ------------------------------------------------'
			executive.updatePlan()
			print '---- updatePlan ------------------------------------------------'
			self.shutdownOnInterrupt()
			self.communicator().waitForShutdown()
		except:
			traceback.print_exc()
			status = 1

		if self.communicator():
			try:
				self.communicator().destroy()
			except:
				traceback.print_exc()
				status = 1



if __name__ == '__main__':
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	elif len(params) == 0:
		params.append('--Ice.Config=config')
	Server( ).main(params)
