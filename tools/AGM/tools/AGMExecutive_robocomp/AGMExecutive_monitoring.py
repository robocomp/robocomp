import copy, sys, traceback, imp, shutil


from agglplanner import *
from agglplanchecker import *

def askPlannerToSolveHierarchicalRule(domainObj, domainModule, domainPath, initWorld, actionName, parameters, excludeList):
	init_path = "/tmp/estadoIntermedio.xml"
	temp_path = "/tmp/estadoIntermedio2.xml"
	initWorld.toXML(init_path)
	estadoIntermedio = domainModule.getTriggers()[actionName](WorldStateHistory([initWorld, set()]), parameters)
	estadoIntermedio.graph.toXML(temp_path)
	quitar_Constantes_Creadas(temp_path)
	"""Generamos el codigo en python para pasarselo directamente al PyPlan"""
	graph = graphFromXMLFile(temp_path)
	outputText = generateTarget(graph)
	ofile = open(temp_path+'.py', 'w')
	ofile.write(outputText)
	ofile.close()
	
	paramsWithoutNew = copy.deepcopy(parameters)
	for param in parameters:
		found = False
		for arule in domainObj.agm.rules:
			if arule.name == actionName:
				if param in arule.lhs.nodes:
					found = True
					break
		if not found:
			del paramsWithoutNew[param]

	rList = []
	aaa = PyPlan(domainObj, domainPath, init_path, domainModule.getHierarchicalTargets()[actionName], '\t', paramsWithoutNew, excludeList, rList, True, temp_path+'.py', copy.deepcopy(domainObj.getInitiallyAwakeRules()))
	if len(aaa.results.getList()) == 0:
		print 'NOOOOOOOOOOOOOOOOOOO'
		gg = graphFromXMLFile(init_path)
		gg.filterGeometricSymbols().toXML("doesntwork.xml")
		shutil.copyfile(temp_path, "doesntwork.py")
		print 'getInitiallyAwakeRules', domainObj.getInitiallyAwakeRules()
		print 'exclude', excludeList
		os._exit(-1)
	return rList

def macroscopicPlanCheck(domainClass, domainPath, init, plan, target):
	currentPlan = AGGLPlannerPlan(str(plan), planFromText=True)
	macroscopicPlan = AGGLPlannerPlan('', planFromText=True)
	
	#
	#
	for action in reversed(currentPlan.data):
		if action.name.startswith('#!'):
			a = copy.deepcopy(action)
			a.name = a.name[2:]
			macroscopicPlan.data.insert(0, AGGLPlannerAction(str(a)))
			break
		else:
			macroscopicPlan.data.insert(0, AGGLPlannerAction(str(action)))
	#
	#
	try:
		p = PyPlanChecker(domainClass, domainPath, init, macroscopicPlan, target, '', verbose=False)
		if not p.valid:
			print 'macroscopicPlanCheck:: ESTE NO FUNCIONA, HAY QUE REPLANIFICAR!!!'
			print macroscopicPlan
			print 'macroscopicPlanCheck:: ESTE NO FUNCIONA, HAY QUE REPLANIFICAR!!!'
			PyPlanChecker(domainClass, domainPath, init, macroscopicPlan, target, '', verbose=True)
			print 'macroscopicPlanCheck:: ESTE NO FUNCIONA, HAY QUE REPLANIFICAR!!!'
			number = 1
			while True:
				planName = 'plan'+str(number).zfill(4)+'.plan'
				with open(planName, "w") as text_file:
					text_file.write(str(macroscopicPlan))
				worldName = 'world'+str(number).zfill(4)+'.xml'
				shutil.copyfile(init, worldName)
				print '-------------------------------------------------------------- Stored in', planName, worldName
				break
			return False
	except:
		traceback.print_exc()
		return False
	
	return True



def AGMExecutiveMonitoring(domainClass, domainPath, init, currentModel, target, plan):
	ret, plan = AGMExecutiveMonitoring_recursive(domainClass, domainPath, init, currentModel, target, plan)
	if not macroscopicPlanCheck(domainClass, domainPath, init, plan, target):
		return False, None
	return ret, plan
	
	

def AGMExecutiveMonitoring_recursive(domainClass, domainPath, init, currentModel, target, plan, rulesToExcludeInSearch=None):
	if rulesToExcludeInSearch == None:
		rulesToExcludeInSearch = set()

	try:
		currentPlan = AGGLPlannerPlan(plan, planFromText=True)
	except:
		traceback.print_exc()
		sys.exit(134)


	if len(currentPlan.data)>0:
		if currentPlan.data[0].name.startswith('#!'):
			currentPlan.removeFirstActionDirect()
			return AGMExecutiveMonitoring_recursive(domainClass, domain, domainPath, init, currentModel, target, currentPlan, rulesToExcludeInSearch)


	domain = imp.load_source('domain', domainPath).RuleSet() # activeRules.py

	###
	### If there is a hierarchical rule in the plan, just monitor the plan up to the goal of such action returning
	### the value obtained from a recursive call
	for index, action in enumerate(currentPlan.data):
		if action.name.startswith('#!'):
			# If the action is the first one, remove the action and return the result of a recursive call
			# to the function, as if such action was not there in the first place
			if index == 0:
				#print 'Barrera en primera linea, quitamos la barrera y retornamos lo que devuelva una llamada recursiva'
				currentPlan.removeFirstActionDirect()
				return AGMExecutiveMonitoring_recursive(domainClass, domain, domainPath, init, currentModel, target, currentPlan, rulesToExcludeInSearch)
			# Otherwise, if the first action starting with '#!' is not the first action, perform the monitorization only with the actions before '#!'
			# and use the execution of the '#!' action as the target
			else:
				#print 'Barrera en medio, tenemos en cuenta un plan parcial y luego le pegamos el viejo'
				# Get the action's name: remove the first two '#!' characters
				actionName = action.name[2:]
				rulesToExcludeInSearch.add(actionName.strip('#!*'))
				# Remove the actions after the #! barrier
				partialPlan = copy.deepcopy(currentPlan)
				partialPlan.data = partialPlan.data[:index]
				planToAppend = copy.deepcopy(currentPlan)
				planToAppend.data = planToAppend.data[index:]
				def partialTarget(graph):
					g = copy.deepcopy(graph)
					try:
						return domain.getHierarchicalTargets()[actionName.strip('#!*')](g, copy.deepcopy(action.parameters))
					except:
						traceback.print_exc()
						sys.exit(1)
				ret2, currentPlan2 = AGMExecutiveMonitoring_recursive(domainClass, domainPath, copy.deepcopy(init), copy.deepcopy(currentModel), partialTarget, copy.deepcopy(partialPlan), rulesToExcludeInSearch)
				#print 'La parte A del plan es'
				#print currentPlan2
				#print 'Si le uno la parte B'
				#print planToAppend
				currentPlan3 = AGGLPlannerPlan()
				currentPlan3.data = currentPlan2.data + planToAppend.data
				if len(currentPlan3.data) > 0:
					if currentPlan3.data[0].name.startswith('#!'):
						del currentPlan3.data[:1]
				#print 'Queda'
				#print currentPlan3
				while True:
					if len(currentPlan3) == 0:
						break
					if not currentPlan3.data[0].name.startswith('#!'):
						break
					currentPlan3.data.pop(0)
				#print 'Deberiamos nuevamente hacer una monitorizacion a este plan, pero bueno.... (1)'
				if len(currentPlan3)>0:
					if currentPlan3.data[0].hierarchical:
						return AGMExecutiveMonitoring_recursive(domainClass, domainPath, copy.deepcopy(init), copy.deepcopy(currentModel), target, copy.deepcopy(currentPlan3), rulesToExcludeInSearch)
				return ret2, currentPlan3

	###
	### Here we should have a plan which has not a hierarchical decomposition barrier. In shuch case it would trigger a 
	### recursive call without the barrier and further actions, and with the goal of the barrier.
	###
	### If possible, try with a plan removing the first action (this is recursively executed until the plan is empty.
	###
	if len(currentPlan)>0:
		try:
			newPlan = copy.deepcopy(currentPlan.removeFirstAction(currentModel))
			ret2, planMonitoring2 = AGMExecutiveMonitoring_recursive(domainClass, domainPath, init, currentModel, target, newPlan, rulesToExcludeInSearch)
			if ret2:
				return ret2, planMonitoring2
		except:
			traceback.print_exc()
			os._exit(-1)
	
	
	###
	### We get to this poing only when all the following conditions are met:
	###
	###  a) there is no hierarchical barrier (#!)
	###  b) the currentPlan doesn't have more actions to remove or the plan without the first action doesn't work
	###
	try:
		p = PyPlanChecker(domainClass, domainPath, init, currentPlan, target, '', verbose=False)
		if not p.valid:
			#print 'AGMExecutiveMonitoring_recursive:: ESTE NO FUNCIONA!!!'
			#print currentPlan
			return False, None
	except:
		traceback.print_exc()
		return False, None


	#print 'AGMExecutiveMonitoring_recursive<<'
	#print 'AGMExecutiveMonitoring_recursive<<'
	#print str(currentPlan).strip()
	#print 'AGMExecutiveMonitoring_recursive<<'
	#print 'AGMExecutiveMonitoring_recursive<<'

	###
	### We get to this poing only when all the following conditions are met:
	###
	###  a) there is no hierarchical barrier (#!)
	###  b) the currentPlan doesn't have more actions to remove or the plan without the first action doesn't work
	###  c) the first action is not hierarchical
	###
	if len(currentPlan) == 0:
		return True, currentPlan
	elif not currentPlan.data[0].hierarchical:
		return True, currentPlan
	else:
		actionName = currentPlan.data[0].name
		rulesToExcludeInSearch.add(actionName.strip('#!*'))
		parameters = currentPlan.data[0].parameters
		print 'Monitoring this plan did not work because the first action ('+actionName+') was hierarchical'
		print 'Not using', rulesToExcludeInSearch
		print '[ [ ['
		print '['
		print '['
		print 'currentPlan'
		print currentPlan
		print '['
		print '['
		print '[ [ ['
		#skPlannerToSolveHierarchicalRule(domainObj,   domainModule, domainPath  initWorld,                   actionName, parameters, excludeList):
		actions = askPlannerToSolveHierarchicalRule(domainClass, domain, domainPath, copy.deepcopy(currentModel), actionName, parameters, list(rulesToExcludeInSearch))
		resultingPlan = AGGLPlannerPlan(str(currentPlan), planFromText=True)
		resultingPlan.data.pop(0)
		hierarchicalActionStr = '#!'+str(actionName)+'@'+str(parameters)
		resultingPlan.data.insert(0, AGGLPlannerAction(hierarchicalActionStr))
		for i in reversed(actions):
			resultingPlan.data.insert(0, AGGLPlannerAction(i))
		print '['
		print '['
		print '[ [ ['
		print ''
		print 'ACHO'
		print 'ACHO'
		print resultingPlan
		print 'ACHO'
		print 'ACHO'
		print 'Deberiamos nuevamente hacer una monitorizacion a este plan, pero bueno.... (2)'
		print ''
		return True, resultingPlan


