import sys, traceback, Ice, subprocess, threading, time, Queue, os, time
import IceStorm

# AGM
sys.path.append('/usr/local/share/agm')

from parseAGGL import *
from generateAGGLPlannerCode import *
from agglplanner import *
from agglplanchecker import *
from agglplanningcache import *

from AGMExecutive_monitoring import *
import xmlModelParser

import unicodedata

import pickle

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
Ice.loadSlice(preStr+"Planning.ice")
import RoboCompAGMCommonBehavior
import RoboCompAGMExecutive
import RoboCompAGMWorldModel
import RoboCompPlanning

import AGMModelConversion


import thriftpy
agglplanner_thrift = thriftpy.load("/usr/local/share/agm/agglplanner.thrift", module_name="agglplanner_thrift")
from thriftpy.rpc import make_client


#
# WARNING: Design issues to take into consideration in the future:
#
# - To properly use hierarchical rules, the monitoring process should properly account for the hierarchical actions that we are currently solving
# - The number of times the domain is parsed, and the times the state-space-expansion code is generated and imported should be reduced to the minimum
#
#
#
#


class PlannerCaller(threading.Thread):
	lastPypyKill = time.time()
	cache = PlanningCache()
	working = False
	plannerExecutionID = 0
	plan = AGGLPlannerPlan('', planFromText=True)
	plannerCallerMutex = threading.RLock()
	pendingJobs = LockableList()


	def __init__(self, executive, agglPath):
		threading.Thread.__init__(self)
		self.executive = executive
		self.agglPath = agglPath
		self.agglplannerclient = executive.agglplannerclient
		self.domainText = open(self.agglPath, 'r').read()
		tempStr = self.domainText
		try: tempStr = unicodedata.normalize('NFKD', tempStr)
		except: pass
		print 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX getDomainIdentifier'
		self.domainId = self.agglplannerclient.getDomainIdentifier(tempStr)

	def setMission(self, targetStr):
		tempStr = targetStr
		try: tempStr = unicodedata.normalize('NFKD', tempStr)
		except: pass
		print 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX getTargetIdentifier'
		self.targetId = self.agglplannerclient.getTargetIdentifier(open(tempStr, 'r').read())

	def setWork(self, currentModel):
		#print 'PlannerCaller::setWork 0'
		# Kill any previous planning process
		while self.plannerCallerMutex.acquire(0)==False:
			#print 'PlannerCaller::setWork 0.5'
			now = time.time()
			elap = (now - self.lastPypyKill)
			if elap > 2.:
				try:
					self.pendingJobs.acquire()
					while True:
						pend = self.pendingJobs.pop()
						print 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX forceStopPlanning'
						self.agglplannerclient.forceStopPlanning(pend)
					self.pendingJobs.release()
				except:
					pass
				self.lastPypyKill = now
			time.sleep(0.1)
		# Set work
		#print 'PlannerCaller::setWork 1'
		self.working = True
		self.currentModel = currentModel
		# End
		self.plannerCallerMutex.release()

	def run(self):
		#print 'run'
		while True:
			#print 'whilie'
			# return if we can't acquire the mutex
			if self.plannerCallerMutex.acquire(False)==False:
				#print '::run: mutex acquire'
				time.sleep(1)
				time.sleep(0.05)
				continue
			# return if there's no work to do
			if not self.working:
				self.plannerCallerMutex.release()
				#print '::run: no self.working'
				time.sleep(1)
				time.sleep(0.05)
				continue
			try:
				self.executiveLoopFunction()
			finally:
				self.plannerCallerMutex.release()

	def executiveLoopFunction(self):
		print 'executiveLoopFunction'
		###
		### Set variables and write files that are of use later
		###
		self.plannerExecutionID+=1
		try: #
			peid = '_'+str(self.plannerExecutionID)
		except:
			print traceback.print_exc()
			print 'There was some problem writing the model to an XML:', "/tmp/lastWorld"+peid+".xml"
			sys.exit(1)

		###
		### Try monitoring
		###
		try:
			#print 'MONITORING??', self.plannerExecutionID
			monitoringResult = self.callMonitoring(peid)
			if monitoringResult:
				print 'PlannerCaller::run: got plan from monitoring'
				self.working = False
				self.executive.gotPlan(self.plan)
				time.sleep(0.05)
				return
			else:
				print 'PlannerCaller::run didn\'t get plan from monitoring'
		except:
			traceback.print_exc()
			return

		###
		### Try the cache
		###
		if False:
			print 'PlannerCaller::run Ask cache'
			try:
				cacheResult = self.cache.getPlanFromFiles(domainPY, worldXML, targetPY)
			except:
				cacheResult = False
			#print 'PlannerCaller::run IGNORING CACHE!!!!'
			#print 'PlannerCaller::run IGNORING CACHE!!!!'
			#cacheResult = False
			#print 'PlannerCaller::run IGNORING CACHE!!!!'
			#print 'PlannerCaller::run IGNORING CACHE!!!!'
			if cacheResult:
				if len(cacheResult[1].strip()) == 0:
					cacheResult = None
			if cacheResult:
				print 'PlannerCaller::run Got plan from cache'
				print '<<<'
				print cacheResult[1]
				print '>>>'
				cacheSuccess = cacheResult[0]
				cachePlan = cacheResult[1]
				lines = cacheResult[1].split('\n')
				if len(''.join(lines).strip()) > 0:
					self.cache.includeFromFiles(domainPY, worldXML, targetPY, "/tmp/result"+peid+".txt", True)
				ofile.close()
				# Get the output
				try:
					self.plan = AGGLPlannerPlan('\n'.join(lines), planFromText=True)
					monitoringResult = self.callMonitoring(peid)
				except:
					traceback.print_exc()
					sys.exit(-1)
				self.working = False
				self.executive.gotPlan(self.plan)
				return

		###
		### Run the planner
		###
		# CALL
		try:
			start = time.time()
			
			print 'PlannerCaller::run calling planner start'
			tempStr = self.currentModel.filterGeometricSymbols().toXMLString()
			try:
				print 'X1', type(tempStr)
				tempStr = str(unicodedata.normalize('NFKD', tempStr))
				print 'X2', type(tempStr)
			except:
				print 'X3', type(tempStr)
				pass
			print 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX startPlanning'
			jobId = self.agglplannerclient.startPlanning(self.domainId, tempStr, self.targetId, [], [])
			print 'PlannerCaller::run got job identifier', jobId
			self.pendingJobs.append(jobId)
			print 'PlannerCaller::run waiting for results...'
			print 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX getPlanningResults'
			result = self.agglplannerclient.getPlanningResults(jobId)
			print 'PlannerCaller::run done calling got', result.plan
			#if len(''.join(lines).strip()) > 0:  # WARNING TODO BUG ERROR FIXME                                 THIS SHOULD BE FIXED TO ENABLE PLAN CACHING
				#self.cache.includeFromFiles(domainPY, worldXML, targetPY, "/tmp/result"+peid+".txt", True)
		except:
			traceback.print_exc()
			print "PlannerCaller::run Can't open plan. We assume a new context was forced"
			print "Quitar el exit"
			sys.exit(-1)
			return
		end = time.time()
		print 'PlannerCaller::run It took', end - start, 'seconds'

		# Get the output
		try:
			print 'A'
			self.plan = AGGLPlannerPlan(str(result.plan), planFromText=True)
			print 'B'
			print 'DIRECT FROM AGGLPlanner\nself.plan:\n', self.plan
			self.callMonitoring(peid)
		except: # The planner was probably killed
			traceback.print_exc()
			return
		self.working = False
		self.executive.gotPlan(self.plan)



	def callMonitoring(self, peid):
		return False
		
		domainPath = '/tmp/domainActive.py'
		init   = '/tmp/lastWorld'+peid+'.xml'
		target = '/tmp/target.py'
		#print 'PERFORMING MONITORING WITH:'
		#print 'domain', domainPath
		#print 'init  ', init
		#print 'target', target
		try:
			try:
				try:
					ret, planMonitoring = AGMExecutiveMonitoring(self.agmData, domainPath, init, self.currentModel.filterGeometricSymbols(), target, AGGLPlannerPlan(self.plan, planFromText=True))
				except:
					print traceback.print_exc()
					print 'PlannerCaller::callMonitoring Error calling AGMExecutiveMonitoring'
					return False
			except (NameError, AttributeError) as e:
				print 'PlannerCaller::callMonitoring There\'s no previous plan yet. It doesn\'t make any sense to use monitoring yet'
				return False
			#print 'AGMExecutiveMonitoring() results:', ret, planMonitoring
			if ret:
				print 'PlannerCaller::callMonitoring Using a forwarded version of the previous plan'
				self.plan = planMonitoring
				return True
			else:
				print 'PlannerCaller::callMonitoring No modified version of the current plan satisfies the goal. Replanning is necessary.'
				return False
		except:
			print traceback.print_exc()
			print 'PlannerCaller::callMonitoring: It didn\'t seem to work.'
			return False

	def ignoreCommentsInPlan(self, plan):
		ret = []
		for i in plan:
			if len(i) > 2:
				if not (i[0] == '#' and i[1] != '!'):
					ret.append(i)
		return ret


class Executive(object):
	def __init__(self, agglPath, initialModelPath, initialMissionPath, doNotPlan, executiveTopic, executiveVisualizationTopic):
		self.doNotPlan = doNotPlan
		self.mutex = threading.RLock()
		self.agents = dict()
		self.plan = AGGLPlannerPlan('', planFromText=True)
		self.modifications = 0
		self.agglplannerclient = make_client(agglplanner_thrift.AGGLPlanner, '127.0.0.1', 6000)
		self.plannerCaller = PlannerCaller(self, agglPath)
		self.plannerCaller.start()
		print '--- setMission ---------------------------------------------'
		self.setMission(initialMissionPath, avoidUpdate=True)
		print '--- setMission ---------------------------------------------'

		# Set proxies
		self.executiveTopic = executiveTopic
		self.executiveVisualizationTopic = executiveVisualizationTopic

		self.agglPath = agglPath
		self.initialModelPath = initialModelPath
		try:
			self.initialModel = xmlModelParser.graphFromXMLFile(initialModelPath)
		except Exception, e:
			print 'Can\'t open ' + initialModelPath + '.'
			os._exit(-1)


		print 'INITIAL MODEL: ', self.initialModel
		self.lastModification = self.initialModel
		print 'initial model version', self.lastModification.version
		self.backModelICE  = None
		self.setAndBroadcastModel(xmlModelParser.graphFromXMLFile(initialModelPath))
		self.worldModelICE = AGMModelConversion.fromInternalToIce(self.currentModel)

	#######################################################################
	#                                                                     #
	# E X E C U T I V E   I N T E R F A C E   I M P L E M E N T A T I O N #
	#                                                                     #
	#                         b e g i n s   h e r e                       #
	#                                                                     #
	#######################################################################
	def activate(self):
		return True

	def deactivate(self):
		return True

	def structuralChangeProposal(self, worldModelICE, sender, log):
		print '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'
		#
		#  H E R E     W E     S H O U L D     C H E C K     T H E     M O D I F I C A T I O N     I S     V A L I D
		#
		print 'Executive::structuralChangeProposal:', sender, log
		# Get structuralChange mutex
		print 'Executive::structuralChangeProposal: structuralChangeProposal acquire() a'
		for iixi in xrange(5):
			gotMutex = self.mutex.acquire(blocking=False)
			if gotMutex == True:
				break
			else:
				if iixi == 4:
					print 'Executive::structuralChangeProposal: structuralChangeProposal acquire() IT WAS LOCKED'
					raise RoboCompAGMExecutive.Locked()
				else:
					time.sleep(0.03)
		print 'Executive::structuralChangeProposal:CURRENT VERSION', self.lastModification.version
		try:
			# Ignore outdated modifications
			if worldModelICE.version != self.lastModification.version:
				print 'Executive::structuralChangeProposal: outdated!??!  self='+str(self.lastModification.version)+'   ice='+str(worldModelICE.version)
				raise RoboCompAGMExecutive.OldModel()
			# Here we're OK with the modification, accept it, but first check if replanning is necessary
			worldModelICE.version += 1
			internalModel = AGMModelConversion.fromIceToInternal_model(worldModelICE, ignoreInvalidEdges=True) # set internal model
			avoidReplanning = False
			# UNCOMMENT THIS!!! WARNING TODO ERROR CUIDADO 
			#try:                                                                                               # is replanning necessary?
				#if internalModel.equivalent(self.lastModification):
					#avoidReplanning = True
			#except AttributeError:
				#pass
			self.lastModification = internalModel                                                              # store last modification
			self.modifications += 1
			self.worldModelICE = worldModelICE
			# Store the model in XML
			#try:
				#internalModel.toXML('modification'+str(self.modifications).zfill(4)+'_'+sender+'.xml')
			#except:
				#print 'There was some problem updating internal model to xml'
			# Set and broadst the model
			try:
				self.setAndBroadcastModel(internalModel)
			except:
				print traceback.print_exc()
				print 'Executive::structuralChangeProposal: There was some problem broadcasting the model'
				sys.exit(-1)
			# Force replanning
			try:
				if not (avoidReplanning or self.doNotPlan):
					self.updatePlan()
			except:
				print traceback.print_exc()
				print 'Executive::structuralChangeProposal: There was some problem updating internal model to xml'
		finally:
			self.mutex.release()
		return

	def symbolUpdate(self, nodeModification):
		self.symbolsUpdate([nodeModification])

	def symbolsUpdate(self, symbols):
		try:
			self.mutex.acquire()
			try:
				for symbol in symbols:
					internal = AGMModelConversion.fromIceToInternal_node(symbol)
					self.currentModel.nodes[internal.name] = copy.deepcopy(internal)
			except:
				print traceback.print_exc()
				print 'Executive::symbolsUpdate: There was some problem with update node'
				sys.exit(1)
			self.executiveTopic.symbolsUpdated(symbols)
		finally:
			self.mutex.release()


	def edgeUpdate(self, edge):
		self.edgesUpdate([edge])

	def edgesUpdate(self, edges):
		try:
			#print 'edgesUpdate acquire() a'
			self.mutex.acquire()
			self.executiveTopic.edgesUpdated(edges)
			#print 'edgesUpdate acquire() z'
			for edge in edges:
				#internal = AGMModelConversion.fromIceToInternal_edge(edge)
				found = False
				for i in xrange(len(self.currentModel.links)):
					if str(self.currentModel.links[i].a) == str(edge.a):
						if str(self.currentModel.links[i].b) == str(edge.b):
							if str(self.currentModel.links[i].linkType) == str(edge.edgeType):
								self.currentModel.links[i].attributes = copy.deepcopy(edge.attributes)
								found = True
				if not found:
					print 'Executive::edgesUpdate: couldn\'t update edge because no match was found'
					print 'Executive::edgesUpdate: edge', edge.a, edge.b, edge.edgeType
		finally:
			self.mutex.release()

	def setMission(self, target, avoidUpdate=False):
		self.mutex.acquire()
		self.targetStr = target
		self.plannerCaller.setMission(self.targetStr)
		self.mutex.release()



	def getModel(self):
		return self.worldModelICE


	def getNode(self, identifier):
		for n in self.worldModelICE.nodes:
			if n.nodeIdentifier == identifier:
				return n

	def getEdge(self, srcIdentifier, dstIdentifier, label):
		for e in self.worldModelICE.edges:
			if e.a == srcIdentifier and e.b == dstIdentifier and e.edgeType == label:
				return e

	def getData(self):
		return self.worldModelICE, self.targetStr, self.plan

	def broadcastPlan(self):
		self.mutex.acquire( )
		try:
			self.publishExecutiveVisualizationTopic()
			print 'broadcastPlan'
			self.sendParams()
		except:
			print traceback.print_exc()
			print 'Executive::broadcastPlan: was some problem broadcasting the plan'
			sys.exit(1)
		self.mutex.release()

	def broadcastModel(self):
		self.mutex.acquire( )
		try:
			print 'Executive::broadcastModel: <<<broadcastinnn'
			self.executiveTopic.structuralChange(AGMModelConversion.fromInternalToIce(self.currentModel))
			print 'Executive::broadcastModel: broadcastinnn>>>'
		except:
			print traceback.print_exc()
			print 'Executive::broadcastModel:There was some problem broadcasting the model'
			sys.exit(1)
		self.mutex.release()
	#########################################################################
	#                                                                     ###
	# E X E C U T I V E   I N T E R F A C E   I M P L E M E N T A T I O N ###
	#                                                                     ###
	#                            e n d s    h e r e                       ###
	#                                                                     ###
	#########################################################################



	def setAgent(self, name, proxy):
		self.agents[name] = proxy


	def setAndBroadcastModel(self, model):
		#print model
		self.currentModel = model
		self.broadcastModel()


	def updatePlan(self):
		self.plannerCaller.setWork(self.currentModel)

	def gotPlan(self, plan):
		self.plan = plan
		# Extract first action
		action = "none"
		parameterMap = dict()
		if len(self.plan) > 0:
			action = self.plan.data[0].name
			parameterMap = self.plan.data[0].parameters

		print 'Executive::gotPlan: action: <'+action+'>  parameters:  <'+str(parameterMap)+'>'
		# Prepare parameters
		params = dict()
		params['action'] = RoboCompAGMCommonBehavior.Parameter()
		params['action'].editable = False
		params['action'].value = action
		params['action'].type = 'string'
		params['plan'] = RoboCompAGMCommonBehavior.Parameter()
		params['plan'].editable = False
		params['plan'].value = str(self.plan)
		params['plan'].type = 'string'
		params['modelversion'] = RoboCompAGMCommonBehavior.Parameter()
		params['modelversion'].editable = False
		params['modelversion'].value = str(self.currentModel.version)
		params['modelversion'].type = 'string'
		for p in parameterMap.keys():
			params[p] = RoboCompAGMCommonBehavior.Parameter()
			params[p].editable = False
			params[p].value = parameterMap[p]
			params[p].type = 'string'
		# Send plan
		self.lastParamsSent = copy.deepcopy(params)
		print 'CURRENT PLAN\n', self.plan
		self.sendParams()
		# Publish new information using the executiveVisualizationTopic
		self.publishExecutiveVisualizationTopic()
		print 'Executive::gotPlan: done'

	#
	#
	def publishExecutiveVisualizationTopic(self):
		try:
			planPDDL = RoboCompPlanning.Plan() # Generate a PDDL-like version of the current plan for visualization
			planPDDL.cost = -2.
			try:
				planPDDL.cost = -1.
				planPDDL.actions = []
				for step in self.plan:
					action = RoboCompPlanning.Action()
					action.name = step.name
					action.symbols = str(step.parameters).translate(None, '{}').split(',')
					planPDDL.actions.append(action)
			except:
				traceback.print_exc()
				print 'Executive::publishExecutiveVisualizationTopic: Error generating PDDL-like version of the current plan'
			self.executiveVisualizationTopic.update(self.worldModelICE, ''.join(open(self.targetStr, "r").readlines()), planPDDL)
		except:
			traceback.print_exc()
			print "Executive::publishExecutiveVisualizationTopic: can't publish executiveVisualizationTopic.update"

	#
	#
	def sendParams(self):
		print 'Executive::sendParams: Send plan to'
		for agent in self.agents:
			print '\t', agent,
			try:
				self.agents[agent].activateAgent(self.lastParamsSent)
			except:
				print '     (can\'t connect to', agent, '!!!)',
			print ''


