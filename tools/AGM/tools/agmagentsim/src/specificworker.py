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

import sys, os, traceback, time
sys.path.append('/usr/local/bin/agmagentsim_py/')

from PySide import *
from genericworker import *


# AGM
sys.path.append('/usr/local/share/agm')

from parseAGGL import *
from generateAGGLPlannerCode import *
from agglplanner import *
from agglplanchecker import *

import xmlModelParser
import AGMModelConversion

class SpecificWorker(GenericWorker):
	events = {}
	Period = 100

	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.timer.start(self.Period)
		self.ui.historyWidgetList.itemClicked.connect(self.historyItemClicked)





	def commitAction(self):
		try:
			self.mutex.lock()
			try:
				plan = self.plan
			except:
				self.ui.currentLabel.setText('no plan yet')
				print 'No plan yet'
				return
			try:
				model = self.internalModel
			except:
				print 'No model yet'
				return
			try:
				action = plan.data[0]
			except IndexError:
				print 'No action to execute'
				return


			print 'Action:   ', action.name
			self.ui.currentLabel.setText(action.name)
			print 'Paramters:', action.parameters
			result = self.triggers[action.name](WorldStateHistory([model, self.triggers.keys()]), action.parameters).graph
			print 'type(result)', type(result)
			resultIce = AGMModelConversion.fromInternalToIce(result)
			self.agmexecutive_proxy.structuralChangeProposal(resultIce, 'agmagentsim', action.name)
		finally:
			self.mutex.unlock()


	def requestPlan(self):
		r = self.agmexecutive_proxy.broadcastPlan()
		print 'request plan:', r

	def setParams(self, params):
		self.ui.commitButton.clicked.connect(self.commitAction)
		self.ui.planButton.clicked.connect(self.requestPlan)

		agmData = AGMFileDataParsing.fromFile(params['DomainPath'])
		agmData.generateAGGLPlannerCode("/tmp/agmagentsimdomain.py", skipPassiveRules=True)
		self.domain = imp.load_source('domain', "/tmp/agmagentsimdomain.py").RuleSet() # activeRules.py
		self.triggers = copy.deepcopy(self.domain.getTriggers()) #get all the active rules of the grammar
		print 'Read actions:', self.triggers.keys()
		return True


	#
	# activateAgent
	def activateAgent(self, prs):
		self.planText = prs['plan'].value.strip()
		self.firstActionText = self.planText.split('\n')[0]
		self.plan = AGGLPlannerPlan(self.planText, planFromText=True)
		print '\nNew plan:'
		print self.planText
		print '\nFirst action:'
		print self.firstActionText
		ev_ident = QtCore.QTime.currentTime().toString("hh:mm:ss.zzz") + " (plan)"
		self.events[ev_ident] = self.planText
		self.ui.historyWidgetList.addItem(ev_ident)
		return True


	@QtCore.Slot()
	def compute(self):
		try:
			plan = self.plan
		except:
			print 'No plan yet'
			self.ui.fullPlanText.setText('')
			self.ui.currentLabel.setText('')
			self.requestPlan()
			return
		try:
			self.mutex.lock()
			model = self.internalModel
		except:
			print 'No model yet'
			w = self.agmexecutive_proxy.getModel()
			self.internalModel = AGMModelConversion.fromIceToInternal_model(w, ignoreInvalidEdges=True)
			return
		finally:
			self.mutex.unlock()

		self.ui.fullPlanText.setText(self.planText)
		if len(self.plan.data) > 0:
			self.ui.currentLabel.setText(self.plan.data[0].name)
		return True


	def historyItemClicked(self, item):
		print 'teta Clicked', item.text()
		text = self.events[item.text()]
		print 'text', text
		self.ui.historyWidgetView.setText(text)


	#
	# structuralChange
	#
	def structuralChange(self, w):
		self.mutex.lock()
		self.internalModel = AGMModelConversion.fromIceToInternal_model(w, ignoreInvalidEdges=True)
		ev_ident = QtCore.QTime.currentTime().toString("hh:mm:ss.zzz") + " (model)"
		self.events[ev_ident] = self.internalModel.toXMLString()
		self.ui.historyWidgetList.addItem(ev_ident)


		self.mutex.unlock()

	#
	# edgesUpdated
	#
	#
	def edgeUpdated(self, modification):
		self.edgesUpdate([modification])
	def edgesUpdated(self, modifications):
		try:
			self.mutex.lock()
			self.executiveTopic.edgesUpdated(edges)
			for edge in edges:
				found = False
				for i in xrange(len(self.internalModel.links)):
					if str(self.internalModel.links[i].a) == str(edge.a):
						if str(self.internalModel.links[i].b) == str(edge.b):
							if str(self.internalModel.links[i].linkType) == str(edge.edgeType):
								self.internalModel.links[i].attributes = copy.deepcopy(edge.attributes)
								found = True
				if not found:
					print 'couldn\'t update edge because no match was found'
					print 'edge', edge.a, edge.b, edge.edgeType
		finally:
			self.mutex.unlock()




	#
	# symbolsUpdated
	#
	def symbolUpdated(self, modification):
		self.symbolsUpdate([modification])
	def symbolsUpdated(self, modifications):
		try:
			self.mutex.acquire()
			try:
				for symbol in symbols:
					internal = AGMModelConversion.fromIceToInternal_node(symbol)
					self.internalModel.nodes[internal.name] = copy.deepcopy(internal)
			except:
				print traceback.print_exc()
				print 'There was some problem with update node'
				sys.exit(1)
			self.executiveTopic.symbolsUpdated(symbols)
		finally:
			self.mutex.release()


	#
	# reloadConfigAgent
	def reloadConfigAgent(self):
		return True

	#
	# setAgentParameters
	def setAgentParameters(self, prs):
		return True


	#
	# getAgentParameters
	def getAgentParameters(self):
		ret = ParameterMap()
		return ret


	#
	# killAgent
	def killAgent(self):
		pass


	#
	# uptimeAgent
	def uptimeAgent(self):
		return 1.0


	#
	# deactivateAgent
	def deactivateAgent(self):
		return True


	#
	# getAgentState
	def getAgentState(self):
		return True





