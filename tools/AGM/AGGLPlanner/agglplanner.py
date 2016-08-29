#!/usr/bin/env pypy

# -*- coding: utf-8 -*-
#
#  -------------------------
#  -----  AGGLPlanner  -----
#  -------------------------
#
#  A free/libre open source AI planner.
#
#  Copyright (C) 2013 - 2014 by Luis J. Manso
#
#  AGGLPlanner is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  AGGLPlanner is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with AGGLPlanner. If not, see <http://www.gnu.org/licenses/>.




#---------------------------------------------------------------------------------------------------------------------------------------------------#
"""@package agglplanner
    @ingroup PyAPI
    This file loads the grammar, the initial state of the world and the GOAL status without changing the files extensions of the grammar and the goal.

    MODE USE:	agglplanner gramatica.aggl init.xml target.py

    Also, we can keep the results in a file with: agglplanner gramatica.aggl init.xml target.py result.plan
"""

import time
import signal
import thread
signal.signal(signal.SIGINT, signal.SIG_DFL)
import sys, traceback, os, re, threading, time, string, math, copy
import collections, imp, heapq
import datetime
sys.path.append('/usr/local/share/agm/')

import xmlModelParser
from AGGL import *
import inspect

from generateAGGLPlannerCode import *


# C O N F I G U R A T I O N
number_of_threads = 0 #4
maxWorldIncrement = 32
maxCost = 2000000000000
stopWithFirstPlan = False
verbose = 1
maxTimeWaitAchieved = 5.
maxTimeWaitLimit = 5000.#1000.



def sumActionCosts(node, last):
	ret = 0
	for t in node.costData[0:last]:
		ret += t[0]
	return ret

def computePlanCost(node):
	ret = 0
	iters = 0
	
	for xcost, xsuccess, xname in node.costData:
		print 'x', xcost, xsuccess, xname
		iters += 1
		ret += xcost + (1-xsuccess)*sumActionCosts(node, iters)
	return ret
	

def quitar_Constantes_Creadas(ficheroMundo):
	constantes = []
	constante = []
	"""Abrimos el fichero del mundo origen y sacamos todos los numeros enteros (variables constantes) del fichero.  Para ello solo nos quedaremos con
	las declaraciones del tipo: <symbol id="1" type="object">"""
	init = open(ficheroMundo)
	linea = init.readline()
	while linea != "":
		if linea.find('<symbol') != -1:
			"""Sacamos la parte id="1" que define el nodo como constante o variable. Minimo, esta parte mide 6
			de longitud. El identificador numerico empieza en la posicion 4.
			Su longitud depende del numero que tenga dentro de las comillas"""
			vector = linea.split()
			for partI in xrange(len(vector)):
				if vector[partI] == '<symbol':
					i = 4
					while i<int(len(vector[partI+1])-1):
						constante.append(vector[partI+1][i])
						i=i+1
			"""Pasamos a entero la cadena con el id del nodo, lo guardamos en el vector de constantes
			y limpiamos las variables usadas."""
			T2 = int(''.join(constante)) 
			constantes.append(T2)
			constante = []
		linea = init.readline()
	init.close()
	
	eliminar = []
	correspondencia = False
	contenido=""
	"""Abrimos el fichero target y vamos comparando todas las lineas  <symbol id="1" type="object">
	con el vector de constantes que hemos sacado antes. Si hay una constante que no esta recogida en el
	vector de constantes es que es nueva y debemos cambiar su nombre para hacerla variable."""
	target = open("/tmp/estadoIntermedio.xml")
	linea = target.readline()
	while linea !="":
		if linea.find('<symbol')!=-1:
			"""Si hemos encontrado una linea que define un nuevo nodo, comparamos el identificador
			del nodo con todas las constantes originales """
			for const in constantes:
				if linea.find('id="'+const.__str__()+'"')!=-1:
					correspondencia = True
			"""Si no hemos encontrado correspondencia ninguna, es que es un nuevo nodo creado a partir
			de la ejecucion de la regla jerarquica. Guardamos su identificador para despues cambiarlo por
			otro valor (lo pasaremos de constante a variable)"""
			if correspondencia==False:
				vector = linea.split()
				i = 4
				while i<int(len(vector[1])-1):
					constante.append(vector[1][i])
					i=i+1
				T2 = int(''.join(constante)) 
				eliminar.append(T2)
				constante = []
		contenido = contenido+linea
		linea = target.readline()
	target.close()

	if len(eliminar)>0:
		"""Si hay elementos a eliminar/cambiar, buscamos sus apariciones y las modificamos"""
		cadenaLimpia=[contenido]
		import re
		target = open("/tmp/estadoIntermedio.xml",'w')    #abrimos el fichero con permisos de escritura
		i=0
		for el in eliminar:
			patter = re.compile('"'+el.__str__()+'"', re.I | re.S)
			cadenaLimpia.append(patter.sub('"AA'+el.__str__()+'"', cadenaLimpia[i]))
			i = i+1
		target.write(cadenaLimpia[len(cadenaLimpia)-1])    #escribimos la cadena ya actualizada y sin la regla jerarquica
		target.close() 


## @brief Method heapsort. This method receives an iterable thing. It stores the iterable thing
# in a list and sorts the list.
# @param iterable is any iterable thing (like a list, a map...)
def heapsort(iterable):
	h = []
	for value in iterable:
		heapq.heappush(h, value)
	return [heapq.heappop(h) for i in range(len(h))]

##@brief This class stored the status and a lock.
class EndCondition(object):
	##@brief Constructor Method. This method saves the status and the lock of the class.
	# @param status it is an optional parameter and, by default, his value is None.
	def __init__(self, status=None):
		## Status of the class --> the status of the final condition
		self.status = status
		## The lock for read and write the status
		self.lock = thread.allocate_lock()

	##@brief Get Method. This method returns the status of the class. To do this, the method needs
	# to lock the other threads. This method reads a critical variable.
	# @retval ret is the status of the class.
	def get(self):
		self.lock.acquire()
		ret = self.status
		self.lock.release()
		return ret

	##@brief Set method. This method changes the value of the status attribute. To do this, the
	# method needs to write in a critical memory zone, so it is necesary to lock it.
	# @param value is the new status
	def set(self, value):
		self.lock.acquire()
		self.status = value
		self.lock.release()

##@brief This class manages the exception of a wrong rule execution.
# It inherits from Excepction.
class WrongRuleExecution(Exception):
	##@brief Constructor method. It saves the data of the Exception
	# @param data anything that gets the information of the Excepction.
	def __init__(self, data):
		## Data of the exception
		self.data = data

	##@brief This method returns the data of the exception.
	# @retval data.
	def __str__(self):
		return self.data

##@brief This class get the name and the parameters of an action that belongs to a plan.
class AGGLPlannerAction(object):
	##@brief Constructor method. It receives (optionally) the contructor method of the object, but, by default, it is empty.
	# @param init is the plan
	# If anything is wrong, this method throws an exception.
	def __init__(self, init=''):
		object.__init__(self)
		## Name of the rule
		self.name = ''
		## Name of the rule parameters (the parameters are in a dictionary)
		self.parameters = dict()
		if len(init)>0:
			#print 'initializing from:', init
			#we save the the list of all the words in the string, using @ as the separator
			parts = init.split('@')
			self.name = parts[0]
			#print self.name
			#print self.name[0], self.name[0] == '*'
			if self.name[0] == '*':
				#print 'hierarchical'
				self.name = self.name[1:]
				self.hierarchical = True
			else:
				self.hierarchical = False
			if self.name[0] != '#':
				#print parts[1]
				self.parameters = eval(parts[1])
		else:
			raise IndexError

	##@brief This method returns the name and the parameters of the action in a string.
	# @retval string with the name of the action and the parameters
	def __str__(self):
		return self.name+'@'+str(self.parameters)

## @brief AGGLPlannerPlan is used as a plan container.
class AGGLPlannerPlan(object):
	## @brief Parametrized constructor.
	# @param init is an optional initialization variable. It can be:
	#	a) string containing a plan;
	#	b) a filename where such string is to be found,
	# 	c) a list of tuples where each tuple contains the name of a rule and a dictionary with the variable mapping of the rule.
	# @param planFromText This parameter indicates whether 'init' is the name of a file (where is stored the plan code) or the text string with the plan code. By default his value is FALSE
	def __init__(self, init='', planFromText=False):
		object.__init__(self)

		## Data of the plan file
		self.data = []
		# IF INIT IS A STRING....
		if type(init) == type(''): # Read plan from file (assuming we've got a file path)
			# If init is a string, we check its length. If there are something in the string, we check
			# what it is:
			if len(init)>0:
				 # If the string is all the plan code, we separate in lines with \n, but,
				 # if the string is the name of the file where is stored all the plan code, we read and save
				 # the file content in a local variable.
				if planFromText:
					lines = init.split("\n") #newline
				else:
					lines = open(init, 'r').readlines() # take the content of a file

				# Now, we check the text to find possible errors.
				# First, we delete the white spaces at the start and the end of each text line.
				# Second, we check if there are any line with only the character \n.
				# Finally, is the line have something that is not a commentary, we save it like a grammar rule
				for line_i in range(len(lines)):
					line = lines[line_i].strip() # take a line of the file content
					while len(line)>0:
						if line[-1]=='\n': line = line[:-1]
						else: break
					if len(line)>0:
						if line[0] != '#':
							try:
								self.data.append(AGGLPlannerAction(line))
							except:
								print 'Error reading plan file', init+". Line", str(line_i)+": <<"+line+">>"
			# If the string hasnt got anything, we dont apply any rule:
			else:
				pass
		# IF INIT IS A LIST
		elif type(init) == type([]):
			# we take each element of the list and we save it as a grammar rule.
			for action in init:
				if action[0][0] != '#':
					#print type(action), len(action), action
					self.data.append(AGGLPlannerAction(action[0]+'@'+str(action[1])))
		#IF INIT IS A COMPLETE PLAN
		else:
			if init.__class__.__name__ == 'AGGLPlannerPlan':
				# we make a copy of the plan
				self.data = copy.deepcopy(init.data)
			else:
				print 'Unknown plan type ('+str(type(init))+')! (internal error)'
				sys.exit(-321)

	## @brief Returns a copy of the plan without the first action assuming the action was executed. It's used for monitorization purposes.
	# @param currentModel Model associated to the current state of the world
	# @retval The resulting version of the plan
	def removeFirstAction(self, currentModel):
		## @internal Create the copy without the first actions.
		c = AGGLPlannerPlan()
		for action in self.data[1:]:
			c.data.append(copy.deepcopy(action))
		## @internal Modify the rest of the actions assuming the first action was executed.
		actionToRemove = self.data[0]
		symbolsInAction = set(actionToRemove.parameters.values())
		nodesInWorld = set(currentModel.nodes.keys())
		created = symbolsInAction.difference(nodesInWorld)
		if len(created) > 0:
			minCreated = min([int(x) for x in created])
			for action in c.data:
				for parameter in action.parameters:
					if int(action.parameters[parameter]) >= minCreated:
						n = str(int(action.parameters[parameter])-len(created))
						action.parameters[parameter] = n
		return c

	def removeFirstActionDirect(self):
		## @internal Create the copy without the first actions.
		c = AGGLPlannerPlan()
		for action in self.data[1:]:
			c.data.append(copy.deepcopy(action))
		return c

	## @brief This method initializes the iterator of the class.
	# @retval the class with the diferent value of the counter
	def __iter__(self):
		## iterator of the data
		self.current = -1
		return self

	## @brief This method counts +1 on the current variable. If the counter overcome the
	# length of the data attribute, it raises an exception. If not, the method returns the
	# data with the index==current+1.
	# @retval the data with the index current+1
	def next(self):
		self.current += 1
		if self.current >= len(self.data):
			raise StopIteration
		else:
			return self.data[self.current]

	##@brief This method returns the information of the graph as a string.
	# @retval a string with the information of the plan graph.
	def __repr__(self):
		## The plan graph
		return self.graph.__str__()

	##@brief this method returns a string with the data array information.
	# @retval ret is the string with all the data
	def __str__(self):
		ret = ''
		for a in self.data:
			ret += a.__str__() + '\n'
		return ret

	##@brief this method returns the length of the data array.
	# @retval an integer that is the length of the data array.
	def __len__(self):
		return len(self.data)

##@brief This class saves all the information of the current graph
class WorldStateHistory(object):
	##@brief Constructor method.
	# @param init initialization variable
	def __init__(self, init):
		object.__init__(self)

		# If INIT IS A GRAPH
		if isinstance(init, list):
			# Graph with the current world status.
			self.graph = copy.deepcopy(init[0])
			# The identifier of the graph that the current graph comes from
			self.parentId = 0
			# The cost of the new graph (as result of execute the heuristic?)
			self.cost = 0
			# The string with all the changes made from the original graph.
			self.history = []
			# CostData
			self.costData = []
			# The identifier of the current graph
			self.nodeId = -1
			# Awaken rules set
			self.awakenRules = init[1]
			# The depth of the current graph.
			self.depth = 0
			# A flag to stop.
			self.stop = False
			# The heuristic score of the current graph
			self.score = 0
		#IF INIT IS A INSTANCE OF WorldStateHistory
		elif isinstance(type(init), type(WorldStateHistory)):
			self.graph = copy.deepcopy(init.graph)
			self.cost = copy.deepcopy(init.cost)
			self.awakenRules = copy.deepcopy(init.awakenRules)
			self.costData = copy.deepcopy(init.costData)
			self.history = copy.deepcopy(init.history)
			self.depth = copy.deepcopy(init.depth)
			self.stop = False
			self.score = 0
		else:
			print 'Internal errorr: Cannot create WorldStateHistory from unexpected object of type', str(type(init))
			sys.exit(1)

	##@brief This method compares two graphs.
	# @param other is the graph with which we compare.
	# @retval an integer that can be 1 if the current graph is greater, -1 if it is smaller, and 0 if both are equals.
	def __cmp__(self, other):
		return self.graph.__cmp__(other.graph)

	##@brief This method calculates the hashcode of the current graph.
	# @retval an integer with the hashcode.
	def __hash__(self):
		return self.graph.__hash__()

	##@brief This method returns the result of comparing two graphs
	# @param other the other graph.
	# @retval a boolean wit the result of the comparison
	def __eq__(self, other):
		return self.graph.__eq__(other.graph)

	##@brief This method returns a string with the information of the current graph.
	def __repr__(self):
		return self.graph.__repr__()

	##@brief This method returns a string with the information of the current graph.
	def __str__(self):
		return self.graph.__str__()

##@brief This method prints on the screen all the information of the new graph: the cost,
# the score, the number of actions and the names of the actions.
def printResult(result):
	print '-----  R  E  S  U  L  T  S  -----'
	if verbose > 0:
		print 'Cost', result.cost
		print 'Score', result.score
		l = 0
		for action in result.history:
			if action[0] != '#':
				l += 1
		print 'Length', l
		print 'Actions\n----------------'
	for action in result.history:
		#if action[0] != '#':
		print action

##@brief This is a class that implements a lockable list.
class LockableList():
	##@brief Constructor Method. It initializes the list and the mutex.
	def __init__(self):
		## The lockable list.
		self.thelist = []
		## The mutex associated to the list.
		self.mutex = threading.RLock()

	##@brief This method return the length of the lockable list.
	# @retval an integer that is the length of the list.
	def __len__(self):
		return len(self.thelist)

	##@brief This method returns the item with the index or keyword a in the list.
	# @param a is the index or the keyword  of the item.
	# @retval the item that corresponds to the index a.
	def __getitem__(self, a):
		return self.thelist.__getitem__(a)

	##@brief This method changes the item for the item b. Is necessary to lock the
	# the method, it changes a critical memory zone.-
	# @param a first item
	# @param b second item.
	def __setitem__(self, a, b):
		self.mutex.acquire()
		self.thelist.__setitem__(a, b)
		self.mutex.release()

	##@brief This method return the iterator of the list.
	# @retval the iterator of the lockable list.
	def __iter__(self):
		return self.thelist.__iter__()

	##@brief This method returns the size (the length) of the list.
	# This method needs acces to a critical memory zone, so it is necessary to
	# lock all the process.
	# @retval ret is an integer that is the size of the list.
	def size(self):
		self.mutex.acquire()
		ret = len(self.thelist)
		self.mutex.release()
		return ret

	##@brief This method makes a pop on the list. First, it acquires the mutex in order to lock all the process.
	# Then It does a pop of the list (using the heappop class). And finally it releases the mutex.
	# @retval the pop of the list.
	def heapqPop(self):
		self.mutex.acquire()
		try:
			ret = heapq.heappop(self.thelist)
		finally:
			self.mutex.release()
		return ret

	##@brief This method insert a value at the top of the list. It needs to acquire the mutex
	# in order to lock all the process.
	# @param value the element to insert.
	def heapqPush(self, value):
		self.mutex.acquire()
		heapq.heappush(self.thelist, value)
		self.mutex.release()

	##@brief The method returns the first element of the list. It needs to acquire the mutex
	# in order to lock all the process. It doesnt remove the element of the list.
	# @retval the first element of the list
	def getFirstElement(self):
		self.mutex.acquire()
		ret = self.thelist[0]
		self.mutex.release()
		return ret

	##@brief This method introduces an element at the end of the list. It needs to acquire the
	# mutex in order to lock all the process.
	# @param v is the new element.
	def append(self, v):
		self.mutex.acquire()
		self.thelist.append(v)
		self.mutex.release()

	##@brief This method lock the mutex (it acquires him)
	def lock(self):
		self.mutex.acquire()

	##@brief This method unlock the mutex (it releases him)
	def unlock(self):
		self.mutex.release()

##@brief This class defines an integer that can be locked and unlocked.
class LockableInteger(object):
	##@brief constructor method. This method initializes the value of the integer and the
	# mutex of the class.
	# @param val is the initial value of the integer (it is an optional parameter).
	def __init__(self, val=0):
		## The value of the integer
		self.value = val
		## The mutex associated to the integer.
		self.mutex = threading.RLock()

	##@brief This method changes the value of the integer. To do this, it needs to
	# take the mutex in order to lock all the process.
	# @param val the new value of the integer.
	def set(self, val):
		self.mutex.acquire()
		self.value = val
		self.mutex.release()

	##@brief This method returns the value of the integer. It needs the mutex.
	# @retval ret is the value of the integer.
	def get(self):
		self.mutex.acquire()
		ret = self.value
		self.mutex.release()
		return ret

	##@brief This method acquires the mutex (it locks him)
	def lock(self):
		self.mutex.acquire()

	##@brief This method releases the mutex (it unlocks him).
	def unlock(self):
		self.mutex.release()

	##@brief This method increases the value of the integer by one.
	# Like always, it needs to take the mutex.
	def increase(self):
		self.mutex.acquire()
		self.value += 1
		self.mutex.release()

## @brief This is the main class. This makes all the process in order to create, check and execute the plan.
class PyPlan(object):
	## @brief The constructor method. This initializes all the attributes of the class and makes the first check of the plan.
	# @param domainAGM  is the file name where is saved the grammar rules.
	# @param domainPath is the file name where is saved the grammar rules.
	# @param init is the XML file where is saved the inital status of the world
	# @param targetPath is the python file where is daved the target status of the world.
	# @param indent The intentation for the output plan
	# @param symbol_mapping mapping that should be used while planning (mainly used internally in recursive rules)
	# @param excludeList grammar rule black list  (those which can't be used for planning)
	# @param resultFile is the optional name of the file where the plan result will be stored.
	# @param descomponiendo: added whe we are descomposing a jerarchical rule
	# @param estadoIntermedio: python file with the intermediate status of the world, after whe apply the jerarchical rule
	# @param awakenRules: the set of rules that are currently available for the planner to find a solution
	def __init__(self, domainAGM, domainPath, init, targetPath, indent, symbol_mapping, excludeList, resultFile, descomponiendo=False, estadoIntermedio='', awakenRules=set()):
		object.__init__(self)
		# Get initial world mdoel
		initWorld = WorldStateHistory([xmlModelParser.graphFromXML(init), domainAGM.getInitiallyAwakeRules()|awakenRules])
		initWorld.nodeId = 0 

		self.symbol_mapping = copy.deepcopy(symbol_mapping) 
		self.excludeList = copy.deepcopy(excludeList)
		self.indent = copy.deepcopy(indent)

		# Get graph rewriting rules
		domain = imp.load_source('domain', domainPath).RuleSet() # activeRules.py
		ruleMap = copy.deepcopy(domain.getRules()) #get all the active rules of the grammar
		for e in excludeList: 
			del ruleMap[e]  # delete the rules in excludeList from ruleMap
		#for r in ruleMap:
			#print 'Using', r

		# Get goal-checking code
		if type(targetPath)== type(''):  # type str
			target = imp.load_source('target', targetPath)
			## This attribute stores the code of the goal status world.
			self.targetCode = target.CheckTarget
		else:
			self.targetCode = targetPath

		# Search initialization
		## This attribute indicates the maximum size that can reach the graph
		self.maxWorldSize = maxWorldIncrement+len(initWorld.graph.nodes.keys())
		## This attribute indicates the minimun cost of the open nodes of the graph
		self.minCostOnOpenNodes = LockableInteger(0)
		## This is the lockable list of all the open nodes.
		self.openNodes = LockableList()
		## This is the lockable list of all the know nodes of the graph.
		self.knownNodes = LockableList()
		## This is the lockable list of all the calculated results.
		self.results = LockableList()
		## This is the lockable integer of all the explored nodes
		self.explored = LockableInteger(0)
		## This is the LockableInteger that stores the better cost of the solution
		self.cheapestSolutionCost = LockableInteger(-1)
		## This is the condition to stop.
		self.end_condition = EndCondition()
		# We save in the list of the open nodes, the initial status of the world
		self.openNodes.heapqPush((0, copy.deepcopy(initWorld)))
		if verbose>1 and indent=='': print 'INIT'.ljust(20), initWorld

		# Create initial state
		if self.symbol_mapping:
			initWorld.score, achieved = self.targetCode(initWorld.graph, self.symbol_mapping)
		else:
			initWorld.score, achieved = self.targetCode(initWorld.graph)

		if achieved:
			# If the goal is achieved, we save the solution in the result list, the
			# solution cost in the cheapest solution cost and we put the end_condition
			# as goal achieved in order to stop the execution.
			self.results.append(initWorld)
			self.cheapestSolutionCost.set(self.results.getFirstElement().cost)
			if self.indent == '':
				self.end_condition.set("GoalAchieved")

		elif number_of_threads>0:
			# But, if the goal is not achieved and there are more than 0 thread running (1, 2...)
			# we creates a list where we will save the status of all the threads, take all
			# the threads (with their locks) and save it in the thread_locks list.
			# Run working threads
			## This attributes is a list where we save all the thread that are running.
			self.thread_locks = []
			threadStatus = LockableList()
			for i in xrange(number_of_threads):
				lock = thread.allocate_lock()
				lock.acquire()
				threadStatus.append(True)
				self.thread_locks.append(lock)
				thread.start_new_thread(self.startThreadedWork, (copy.deepcopy(ruleMap), lock, i, threadStatus))
			# Wait for the threads to stop
			for lock in self.thread_locks:
				lock.acquire()
		else:
			# If the solution is not achieved and there arent any thread to execute the programm
			# we stop it and show an error message.
			self.startThreadedWork(ruleMap)

		# We make others checks over the end_condition:
		#	-- If the end condition is wrong.
		#	-- or the end condition exceeded the maximum cost
		#	-- or we have found the best overcome.
		#	-- or we have achieved the goal
		#	-- or we have exceeded the maximum time limit
		#	-- or the end condition doesnt have any message
		# we print the correspond message
		if self.end_condition.get() == "IndexError":
			if verbose > 0: print 'End: state space exhausted'
		elif self.end_condition.get() == "MaxCostReached":
			if verbose > 0: print 'End: max cost reached'
		elif self.end_condition.get() == "BestSolutionFound":
			if verbose > 0: print 'End: best solution found'
		elif self.end_condition.get() == "GoalAchieved":
			if self.indent == '' and verbose > 0: print 'End: goal achieved'
		elif self.end_condition.get() == "TimeLimit":
			if verbose > 0: print 'End: TimeLimit'
		elif self.end_condition.get() == None:
			if verbose > 0: print 'NDD:DD:D:EWJRI'
		else:
			print 'UNKNOWN ERROR'
			print self.end_condition.get()

		if len(self.results)==0: # If the length of the list is zero, it means that we have not found a plan.
			if verbose > 0: print 'No plan found.'
		else: # But, if the length is greater than zero, it means that we have found a plan.
			min_idx = 0
			for i in range(len(self.results)): # We go over all the actions of the plan, and we look for the best solution (the minimun cost).
				if self.results[i].cost < self.results[min_idx].cost:
					min_idx = i
			i = min_idx
			
			if self.indent=='' and verbose > 0: print 'Got', len(self.results),' plans!'
				
		
			try:
				plann = AGGLPlannerPlan([xx.split('@') for xx in self.results[i].history])
				n = copy.deepcopy(plann)
				while len(self.results[i].history)>0:
					n = n.removeFirstAction(initWorld.graph)
					#n = n.removeFirstActionDirect()
					try:
						#print 'QUITADA REGLA: ', self.results[i].history[0], '\nRESTO DEL PLAN: ', n
						"""ANIADIDO DE MERCEDES, YEAH!
						Si estamos descomponiendo una regla jerarquica (estamos buscando un plan para
						llegar al estado que nos indica la regla jerarquica) debemos crear el estado intermedio
						generado por la regla jerarquica y llamar con el al PyPlanChecker"""
						if descomponiendo==True:
							#print 'Hay que crear estado intermedio'
							from agglplanchecker import PyPlanChecker
							check = PyPlanChecker(domainAGM, domainPath, init, n, estadoIntermedio,symbol_mapping, verbose=False)
							
						else:
							#print 'No hay que crear estado intermedio'
							from agglplanchecker import PyPlanChecker
							check = PyPlanChecker(domainAGM, domainPath, init, n, targetPath,symbol_mapping, verbose=False)
					except:
						print 'Excepction!!'
						traceback.print_exc()
						break
					if check.achieved:
						print  '  (removed)', self.results[i].history[0]
						self.results[i].history = self.results[i].history[1:]
						plann = copy.deepcopy(n)
					else:
						print  '  (not removed)', self.results[i].history[0]
						break
				if descomponiendo==False:
					print 'ORIGINAL PLAN: ', self.results[i].cost
					for action in self.results[i].history:
						print '    ', action
			except:
				traceback.print_exc()
				pass
			rList = []
			planConDescomposicion = False
			if len(self.results[i].history) > 0:
				action = self.results[i].history[0]
				ac = AGGLPlannerAction(action)
				if ac.hierarchical:
					self.excludeList.append(ac.name)
					paramsWithoutNew = copy.deepcopy(ac.parameters)
					for param in ac.parameters:
						found = False
						for arule in domainAGM.agm.rules:
							if arule.name == ac.name:
								if param in arule.lhs.nodes:
									found = True
									break
						if not found:
							#print 'removing fixed goal symbol', param
							del paramsWithoutNew[param]
							#paramsWithoutNew[param] = str('v')+str(paramsWithoutNew[param])
					#print paramsWithoutNew
					print '\nDecomposing hierarchical rule ', ac.name, paramsWithoutNew
					"""ANIADIDO DE MERCEDES:
					Creamos estado intermedio, primero lo creamos en fichero .xml para poder quitarle los nuevos nodos creados
					al aplicar la regla jerarquica"""
					for estadoIntermedio in ruleMap[ac.name](initWorld): print ''
					estadoIntermedio.graph.toXML("/tmp/estadoIntermedio.xml")
					"""Quitamos los nodos constantes creados por la regla jerarquica: los volvemos variables para evitar
					errores cuando se genere el codigo target en python."""
					quitar_Constantes_Creadas(init)
					"""Generamos el codigo en python para pasarselo directamente al PyPlan"""
					graph = graphFromXML("/tmp/estadoIntermedio.xml")
					outputText = generateTarget(graph)
					ofile = open("/tmp/estadoIntermedio.py", 'w')
					ofile.write(outputText)
					ofile.close()
					"""Ponemos una bandera para pintar despues el plan completa una vez descompuesta la primera regla jerarquica"""
					planConDescomposicion = True
					aaa = PyPlan(      domainAGM, domainPath, init, domain.getHierarchicalTargets()[ac.name], indent+'\t', paramsWithoutNew, self.excludeList, rList, True, "/tmp/estadoIntermedio.py", copy.deepcopy(self.results[i].awakenRules|awakenRules))
					#if type(resultFile) == type([]):
						#resultFile = rList + resultFile[1:]
					#print self.indent
					self.results[i].history = self.results[i].history[1:]
				else:
					planConDescomposicion = False
				
			#printResult(self.results[i]) #the best solution
			total = rList + self.results[i].history
			if resultFile != None:
				for action in total:
					if type(resultFile) == type([]):
						resultFile.append(action)
					else:
						resultFile.write(str(action)+'\n')			
			if descomponiendo == False and planConDescomposicion == True:
				print 'FINAL PLAN WITH: ',len(total),' ACTIONS:'
				for action in total:
					print '    ',action
			
			if self.indent=='' and verbose > 0: print "----------------\nExplored", self.explored.get(), "nodes"
		
	##@brief This method starts the execution of program threads
	# @param ruleMap all the actives rules of the grammar
	# @param lock this is the clench of the each thread.
	# @param i is the index of the thread
	# @param threadPoolStatus this is the status of the thread. The threads have three possible status:
	#	1) On hold because they are waiting for another thread finishes his execution.
	#	2) Active. They are expanding a node.
	#	3) All the threads are idle because all of them have finished his executions.
	def startThreadedWork(self, ruleMap, lock=None, i=0, threadPoolStatus=None):
		if lock == None:
			# If there isnt any lock, we create one and we give it to the thread.
			lock = thread.allocate_lock()
			lock.acquire()

		if threadPoolStatus != None:
			# If the thread status is different of none, we lock the
			# code and put the status of the i thread.
			threadPoolStatus.lock()
			threadPoolStatus[i] = True # el hilo esta en marcha.
			threadPoolStatus.unlock()
		# We take the initial time.
		timeA = datetime.datetime.now()
		
		while True:
			# Again, we take the time and we calculated the elapsed time
			timeB = datetime.datetime.now()
			timeElapsed = float((timeB-timeA).seconds) + float((timeB-timeA).microseconds)/1e6
			# We take the length of the result list.
			nResults = self.results.size()
			# Check if we should give up because it already took too much time
			if timeElapsed > maxTimeWaitLimit or (timeElapsed > maxTimeWaitAchieved and nResults > 0):
				if nResults>0:
					self.end_condition.set("GoalAchieved")
				else:
					self.end_condition.set("TimeLimit")
				lock.release()
				return
			# Else, proceed...
			# Try to pop a node from the queue
			try:
				# We take the head of the openNodes list: the first open node.
				head = self.openNodes.heapqPop()[1] # P O P   POP   p o p   pop		
				if threadPoolStatus:
					threadPoolStatus.lock()
					threadPoolStatus[i] = True # We say that the thread i is active.
					threadPoolStatus.unlock()
				self.knownNodes.append(head)
			except:
				#traceback.print_exc()
				if not threadPoolStatus:
					self.end_condition.set("IndexError")
					lock.release()
					return
				time.sleep(0.001)
				threadPoolStatus.lock()
				threadPoolStatus[i] = False
				if not True in threadPoolStatus:
					self.end_condition.set("IndexError")
					lock.release()
					return
				threadPoolStatus.unlock()
				continue

			# Update 'minCostOnOpenNodes', so we can stop when the minimum cost in the queue is bigger than one in the results
			self.updateMinCostOnOpenNodes(head.cost)			
			# Check if we got to the maximum cost or the minimum solution
			if head.cost > maxCost:
				self.end_condition.set("MaxCostReached")
				lock.release()
				return
			elif self.results.size()>0 and head.cost>self.cheapestSolutionCost.value and stopWithFirstPlan:
				self.end_condition.set("GoalAchieved")
				lock.release()
				return
			if verbose>5: print 'Expanding'.ljust(5), head
			for k in ruleMap:
				# Iterate over rules and generate derivates
				if True:# k in head.awakenRules:
					for deriv in ruleMap[k](head):
						self.explored.increase()
						if self.symbol_mapping:
							deriv.score, achieved = self.targetCode(deriv.graph, self.symbol_mapping)
						else:
							deriv.score, achieved = self.targetCode(deriv.graph)
						if achieved:
							self.results.append(deriv)
							if stopWithFirstPlan:
								self.end_condition.set("GoalAchieved")
								lock.release()
								return
							# Compute cheapest solution
							self.updateCheapestSolutionCostAndCutOpenNodes(self.results[0].cost)
						self.knownNodes.lock()
						notDerivInKnownNodes = not self.computeDerivInKnownNodes(deriv)
						self.knownNodes.unlock()
						if notDerivInKnownNodes:
							if deriv.stop == False:
								if len(deriv.graph.nodes.keys()) <= self.maxWorldSize:
									self.openNodes.heapqPush( (float(deriv.cost)-10.*float(deriv.score), deriv) ) # The more the better TAKES INTO ACCOUNT COST AND SCORE
			if verbose > 0:
				doIt=False
				nowNow = datetime.datetime.now()
				try:
					elap = (nowNow-self.lastTime).seconds + (nowNow-self.lastTime).microseconds/1e6
					doIt = elap > 3
				except:
					self.lastTime = datetime.datetime.now()
					doIt = True
				if doIt:
					self.lastTime = nowNow
					#try:
						#if self.indent == '':
					print str(int(timeElapsed)).zfill(10)+','+str(len(self.openNodes))+','+str(len(self.knownNodes))+','+str(head.score)
						#rrrr = heapsort(self.openNodes)
						#print 'OpenNodes', len(rrrr), "(HEAD cost:"+str(head.cost)+"  depth:"+str(head.depth)+"  score:"+str(head.score)+")"
						#if len(self.openNodes) > 0:
							#print 'First['+str(rrrr[ 0][0])+'](cost:'+str(rrrr[ 0][1].cost)+', score:'+str(rrrr[ 0][1].score)+', depth:'+str(rrrr[ 0][1].depth)+')'
							#print  'Last['+str(rrrr[-1][0])+'](cost:'+str(rrrr[-1][1].cost)+', score:'+str(rrrr[-1][1].score)+', depth:'+str(rrrr[-1][1].depth)+')'
						#else:
							#print 'no open nodes'
					#except:
						#traceback.print_exc()

	def updateCheapestSolutionCostAndCutOpenNodes(self, cost):
		self.cheapestSolutionCost.lock()
		if cost >= self.cheapestSolutionCost.get():
			self.cheapestSolutionCost.unlock()
		else:
			self.openNodes.lock()
			self.cheapestSolutionCost.set(cost)

			newOpenNodes = LockableList()
			for node in self.openNodes:
				if node[1].cost < cost:
					newOpenNodes.heapqPush(node)
			self.openNodes.thelist = newOpenNodes.thelist

			self.openNodes.unlock()
			self.cheapestSolutionCost.unlock()

	def updateMinCostOnOpenNodes(self, cost):
		self.minCostOnOpenNodes.lock()
		self.openNodes.lock()
		if cost < self.minCostOnOpenNodes.value:
			if self.getNumberOfOpenNodes()==0:
				self.minCostOnOpenNodes.value = 0
			else:
				self.minCostOnOpenNodes.value = self.firstOpenNode()[1].cost
				for n in self.openNodes:
					if n[1].cost < self.minCostOnOpenNodes.value:
						self.minCostOnOpenNodes.value = n[1].cost
		self.openNodes.unlock()
		self.minCostOnOpenNodes.unlock()

	def  computeDerivInKnownNodes(self, deriv):
		for other in self.knownNodes:
			if deriv == other:
				if other.cost <= deriv.cost:
					return True
		return False

if __name__ == '__main__': # program domain problem result
	#from pycallgraph import *
	#from pycallgraph.output import GraphvizOutput
	#graphviz = GraphvizOutput()
	#graphviz.output_file = 'basic.png'
	if True:
	#with PyCallGraph(output=graphviz):
		from parseAGGL import AGMFileDataParsing   #AGMFileDataParsing en fichero parseAGGL.py
		t0 = time.time()

		if len(sys.argv)<5:
			print 'Usage\n\t', sys.argv[0], ' domain.aggl activeRules.py init.xml target.xml.py [result.plan]'

		else:
			domainAGM = AGMFileDataParsing.fromFile(sys.argv[1]) #From domain.aggl
			domainPath = sys.argv[2] # Get the activeRules.py path
			init = sys.argv[3]       # Get the inital model or world.
			targetPath = sys.argv[4] # Get the target model or world.
			resultFile = None        
			if len(sys.argv)>=6: resultFile = open(sys.argv[5], 'w')

			p = PyPlan(domainAGM, domainPath, init, targetPath, '', dict(), [], resultFile)
		print 'Tiempo Total: ', (time.time()-t0).__str__()

