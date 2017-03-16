import sys, traceback, Ice, subprocess, threading, time, Queue, os, time
import IceStorm

# AGM
sys.path.append('/usr/local/share/agm')

from parseAGGL import *
from generateAGGLPlannerCode import *
from agglplanner import *
from agglplanchecker import *
from agglplanningcache import *

import xmlModelParser

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


#ret, stepsFwd, planMonitoring =
def AGMExecutiveMonitoring(domainClass, domainPath, init, currentModel, target, plan, stepsFwd=0):
	try:
		currentPlan = AGGLPlannerPlan(plan)
	except:
		traceback.print_exc()
		sys.exit(134)

	try:
		ret2 = False
		if len(currentPlan)>0:
			try:
				#print 'Trying one step ahead...', stepsFwd
				newPlan = currentPlan.removeFirstAction(currentModel)
				ret2, stepsFwd2, planMonitoring2 = AGMExecutiveMonitoring(domainClass, domainPath, init, currentModel, target, newPlan, stepsFwd+1)
			except:
				print steps, 'steps ahead did not work'
				traceback.print_exc()
				ret2 = False
		if ret2:
			print 'monitoring ok', stepsFwd
			return ret2, stepsFwd2, planMonitoring2
		else:
			try:
				#print 'CHECK with a plan of', len(plan), 'steps,', stepsFwd, 'forward'
				#print plan
				p = PyPlanChecker(domainClass, domainPath, init, currentPlan, target, '', verbose=False)
				if p.valid:
					#print 'GOT PLAN FROM MONITORING!!!'
					#print currentPlan
					return True, stepsFwd, currentPlan
				else:
					#print 'doesn\'t work'
					return False, 0, None
			except:
				traceback.print_exc()
				return False, 0, None
	except:
		traceback.print_exc()
		sys.exit(4991)
	traceback.print_exc()
	sys.exit(692)


class PlannerCaller(threading.Thread):
	def __init__(self, executive, agglPath):
		threading.Thread.__init__(self)
		self.lastPypyKill = time.time()
		self.cache = PlanningCache()
		self.plannerExecutionID = 0
		self.executive = executive
		self.working = False
		self.plannerCallerMutex = threading.RLock()
		self.agglPath = agglPath
		self.agmData = AGMFileDataParsing.fromFile(self.agglPath)
		self.agmData.generateAGGLPlannerCode("/tmp/domainActive.py", skipPassiveRules=False)
		self.agmData.generateAGGLPlannerCode("/tmp/domainPasive.py", skipPassiveRules=True)

	def setWork(self, currentModel):
		print 'PlannerCaller::setWork 0'
		# Kill any previous planning process
		while self.plannerCallerMutex.acquire(0)==False:
			print 'PlannerCaller::setWork 0.5'
			now = time.time()
			elap = (now - self.lastPypyKill)
			if elap > 2.:
				try: subprocess.call(["killall", "-9", "pypy"])
				except: pass
				self.lastPypyKill = now
			time.sleep(0.1)
		# Set work
		print 'PlannerCaller::setWork 1'
		self.working = True
		self.currentModel = currentModel
		# End
		self.plannerCallerMutex.release()

	def run(self):
		while True:
			# Continue if we can't acquire the mutex
			if self.plannerCallerMutex.acquire(False)==False:
				#print '::run: mutex acquire'
				#time.sleep(1)
				time.sleep(0.05)
				continue
			# Continue if there's no work to do
			if not self.working:
				self.plannerCallerMutex.release()
				#print '::run: no self.working'
				#time.sleep(1)
				time.sleep(0.05)
				continue
			try:
				print 'MONITOREANDO??'
				print 'MONITOREANDO??'
				stored, stepsFwd = self.callMonitoring('_'+str(self.plannerExecutionID))
				if stored:
					self.working = False
					self.executive.gotPlan(self.plan)
					self.plannerCallerMutex.release()
					time.sleep(0.05)
					continue
			except: # The planner was probably killed
				traceback.print_exc()
				return


			print 'PlannerCaller::run 2'
			try:
				# Run planner
				start = time.time()
				#PRE
				try:
					self.plannerExecutionID+=1
					peid = '_'+str(self.plannerExecutionID)
					self.currentModel.filterGeometricSymbols().toXML("/tmp/lastWorld"+peid+".xml")
				except:
					print traceback.print_exc()
					print 'There was some problem writing the model to an XML:', "/tmp/lastWorld"+peid+".xml"
					sys.exit(1)
				# CALL
				argsss = ["agglplanner", self.agglPath, "/tmp/domainActive.py", "/tmp/lastWorld"+peid+".xml", "/tmp/target.py", "/tmp/result"+peid+".txt"]
				print 'Ask cache'
				try:
					cacheResult = self.cache.getPlanFromFiles(argsss[2], argsss[3], argsss[4])
				except:
					cacheResult = False
				cacheResult = False
				if cacheResult:
					if len(cacheResult[1].strip()) == 0:
						cacheResult = None
				if cacheResult:
					print 'Got plan from cache'
					print '<<<'
					print cacheResult[1]
					print '>>>'
					cacheSuccess = cacheResult[0]
					cachePlan = cacheResult[1]
					lines = cacheResult[1].split('\n')
				else:
					stored = False
					try:
						print 'Not found in cache, checking monitoring...'
						stored, stepsFwd = self.callMonitoring(peid)
						print 'callMonitoring', stored, stepsFwd
					except:
						print 'Can\'t get plan from monitoring'
						pass
					print 'Running the planner?', stored==False


					stored = False
					if stored == False:
						try:
							print 'argss', argsss
							print 'call', argsss
							subprocess.call(argsss)
							print 'done calling', argsss
						except:
							traceback.print_exc()
						#CONTINUE?                       Probably it's better just to try to open the file and consider it was killed if there's no result file...
						end = time.time()
						print 'It took', end - start, 'seconds'
						print 'includeFromFiles: ', argsss[2], argsss[3], argsss[4], "/tmp/result"+peid+".txt", True
						try:
							ofile = open("/tmp/result"+peid+".txt", 'r')
						except:
							print "Can't open plan. We assume a new context was forced"
							continue
						try:
							lines = self.ignoreCommentsInPlan(ofile.readlines())
							if len(''.join(lines).strip()) > 0:
								self.cache.includeFromFiles(argsss[2], argsss[3], argsss[4], "/tmp/result"+peid+".txt", True)
							ofile.close()
						except:
							print 'Weird error xx'
							pass
				# Get the output
				try:
					self.plan = AGGLPlannerPlan('\n'.join(lines), planFromText=True)
					print 'self.plan', self.plan
					stored, stepsFwd = self.callMonitoring(peid)
				except: # The planner was probably killed
					traceback.print_exc()
					return
				self.working = False
				self.executive.gotPlan(self.plan)
			finally:
				self.plannerCallerMutex.release()



	def callMonitoring(self, peid):
		stored = False
		stepsFwd = 0
		print 'Trying with the last steps of the current plan...'
		domainPath = '/tmp/domainActive.py'
		init   = '/tmp/lastWorld'+peid+'.xml'
		target = '/tmp/target.py'
		try:
			try:
				ret, stepsFwd, planMonitoring = AGMExecutiveMonitoring(self.agmData, domainPath, init, self.currentModel.filterGeometricSymbols(), target, AGGLPlannerPlan(self.plan))
			except:
				print 'There\'s no previous plan yet. It doesn\'t make any sense to use monitoring yet'
				return False, 0, None
			#print 'bb'
			#print ret, stepsFwd, planMonitoring
			if ret:
				print 'Using a ', stepsFwd, 'step forwarded version of the previous plan'
				stored = True
				self.plan = planMonitoring
			else:
				print 'No modified version of the current plan satisfies the goal. Replanning is necessary.'
				stored = False
		except:
			print traceback.print_exc()
			print 'It didn\'t seem to work.'
			stored = False
		print 'done callMonitoring', stored, stepsFwd
		return stored, stepsFwd

	def ignoreCommentsInPlan(self, plan):
		ret = []
		for i in plan:
			if len(i) > 0:
				if i[0] != '#':
					ret.append(i)
		return ret


class Executive(object):
	def __init__(self, agglPath, initialModelPath, initialMissionPath, doNotPlan, executiveTopic, executiveVisualizationTopic):
		self.doNotPlan = doNotPlan
		self.mutex = threading.RLock()
		self.agents = dict()
		self.plan = None
		self.modifications = 0
		self.plannerCaller = PlannerCaller(self, agglPath)
		self.plannerCaller.start()

		# Set proxies
		self.executiveTopic = executiveTopic
		self.executiveVisualizationTopic = executiveVisualizationTopic

		self.agglPath = agglPath
		self.initialModelPath = initialModelPath
		try:
			self.initialModel = xmlModelParser.graphFromXML(initialModelPath)
		except Exception, e:
			print 'Can\'t open ' + initialModelPath + '.'
			sys.exit(-1)


		print 'INITIAL MODEL: ', self.initialModel
		self.lastModification = self.initialModel
		print 'initial model version', self.lastModification.version
		self.backModelICE  = None
		self.setAndBroadcastModel(xmlModelParser.graphFromXML(initialModelPath))
		self.worldModelICE = AGMModelConversion.fromInternalToIce(self.currentModel)
		print '--- setMission ---------------------------------------------'
		self.setMission(initialMissionPath, avoidUpdate=True)
		print '--- setMission ---------------------------------------------'

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
		#
		#  H E R E     W E     S H O U L D     C H E C K     T H E     M O D I F I C A T I O N     I S     V A L I D
		#
		print 'structuralChangeProposal', sender, log
		# Get structuralChange mutex
		print 'structuralChangeProposal acquire() a'
		for iixi in xrange(5):
			gotMutex = self.mutex.acquire(blocking=False)
			if gotMutex == True:
				break
			else:
				if iixi == 4:
					print 'structuralChangeProposal acquire() IT WAS LOCKED'
					raise RoboCompAGMExecutive.Locked()
				else:
					time.sleep(0.03)
		print 'CURRENT VERSION', self.lastModification.version
		try:
			print 'structuralChangeProposal acquire() z'
			# Ignore outdated modifications
			if worldModelICE.version != self.lastModification.version:
				print 'outdated!??!  self='+str(self.lastModification.version)+'   ice='+str(worldModelICE.version)
				raise RoboCompAGMExecutive.OldModel()
			print 'inside'
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
				print 'There was some problem broadcasting the model'
				sys.exit(-1)
			# Force replanning
			try:
				if not (avoidReplanning or self.doNotPlan):
					self.updatePlan()
			except:
				print traceback.print_exc()
				print 'There was some problem updating internal model to xml'
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
				print 'There was some problem with update node'
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
					print 'couldn\'t update edge because no match was found'
					print 'edge', edge.a, edge.b, edge.edgeType
		finally:
			self.mutex.release()

	def setMission(self, target, avoidUpdate=False):
		self.mutex.acquire()
		self.targetStr = target
		try:
			temp = AGMFileDataParsing.targetFromFile(target)
			self.target = generateTarget_AGGT(temp)
			ofile = open("/tmp/target.py", 'w')
			ofile.write(self.target)
			ofile.close()
			if not avoidUpdate:
				print 'do update plan'
				self.updatePlan()
		except:
			print traceback.print_exc()
			print 'There was some problem setting the mission'
			sys.exit(1)
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
			self.sendParams()
		except:
			print traceback.print_exc()
			print 'There was some problem broadcasting the plan'
			sys.exit(1)
		self.mutex.release()

	def broadcastModel(self):
		self.mutex.acquire( )
		try:
			print '<<<broadcastinnn'
			print self.currentModel.filterGeometricSymbols()
			self.executiveTopic.structuralChange(AGMModelConversion.fromInternalToIce(self.currentModel))
			print 'broadcastinnn>>>'
		except:
			print traceback.print_exc()
			print 'There was some problem broadcasting the model'
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

		print 'action: <'+action+'>  parameters:  <'+str(parameterMap)+'>'
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
		for p in parameterMap.keys():
			params[p] = RoboCompAGMCommonBehavior.Parameter()
			params[p].editable = False
			params[p].value = parameterMap[p]
			params[p].type = 'string'
		# Send plan
		self.lastParamsSent = copy.deepcopy(params)
		self.sendParams()
		# Publish new information using the executiveVisualizationTopic
		self.publishExecutiveVisualizationTopic()
		print 'done'

	#
	#
	def publishExecutiveVisualizationTopic(self):
		try:
			print self.plan
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
				print 'Error generating PDDL-like version of the current plan'
			self.executiveVisualizationTopic.update(self.worldModelICE, ''.join(open(self.targetStr, "r").readlines()), planPDDL)
		except:
			traceback.print_exc()
			print "can't publish executiveVisualizationTopic.update"

	#
	#
	def sendParams(self):
		print 'Send plan to'
		for agent in self.agents:
			print '\t', agent,
			try:
				self.agents[agent].activateAgent(self.lastParamsSent)
			except:
				print '     (can\'t connect to', agent, '!!!)',
			print ''


