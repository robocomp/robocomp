#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------------
#  -----  AGGLPlanChecker  -----
#  -----------------------------
#
#  Almost a free/libre AI planner.
#
#  Copyright (C) 2013-2014 by Luis J. Manso
#
#  AGGLPlanChecker is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  AGGLPlanChecker is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with AGGLPlanner. If not, see <http://www.gnu.org/licenses/>.




#---------------------------------------------------------------------------------------------------------------------------------------------------#
"""@package agglplanchecker
    @ingroup PyAPI
    This file contains agglplannerchecker and AGGLPlanChecker, a stand-alone program and a class, respectively, used to verify plans.
"""



# Python distribution imports
#import signal
#signal.signal(signal.SIGINT, signal.SIG_DFL)
import sys, traceback, os, re, threading, time, string, math, copy
import collections, imp, heapq

sys.path.append('/usr/local/share/agm/')

import xmlModelParser
from AGGL import *
import inspect
from agglplanner import *
from parseAGGL import *
from generateAGGLPlannerCode import *
from agglplannerplan import *

##@brief This class is responsible for checking the plan that is generated in python.
class AGGLPlanChecker(object):
	##@brief Constructor method: It initializes all class attributes and checks the plan
	# @param agmData AGMFileDataParsing instance
	# @param domainPath Python version of the grammar
	# @param init XML-version of the initial world state
	# @param planPath Path of the plan
	# @param targetPath Python-version of the target state OR an instance of a function
	# @param symbolMapping mapping that should be used while planning (mainly used internally in recursive rules)
	# @param resultPath is the path of the result is stored. By default it's empty
	# @param verbose is a parameter that shows additional information

	def __init__(self, parsedDomain, domainModule, init, plan, target, symbolMapping=dict(), verbose=False):
		object.__init__(self)
		## We get the initial world model graph
		self.initWorld  = init
		## We save the grammar rules
		self.domain = domainModule
		# Get goal-checking code
		self.target = target
		## We get the plan code
		self.plan = AGGLPlannerPlan(plan)
		self.verbose = verbose
	def run(self):
		# Apply plan
		if self.verbose: print "AGGLPlanChecker applying plan"
		try:
			world = copy.deepcopy(self.initWorld) # we copy the initial world status.
			#if self.verbose: print world
			line = 0 # This is the actions line counter. It keeps track of the lines of actions contained in a plan
			if self.verbose: print '<<plan\n', self.plan, '\nplan>>'
			# We check all the actions in a plan.
			for action in self.plan:
				if self.verbose: print 'Executing action', line,' ',action
				line += 1
				# We check that the actions parameters are into the world.
				for p in action.parameters.keys():
					if not action.parameters[p] in world.graph.nodes.keys():
						for r in agmData.agm.rules:
							if r.name == action.name:
								rhs = r.rhs
						if not (p in rhs.nodes.keys()):
							# If the parameter doesnt exit in the world nor the RHS of the rule, we raise an exception.
							# Just to warn we're receiving useless parameters
							raise WrongRuleExecution("Parameter '"+action.parameters[p]+"' (variable '"+p+"') doesn't exist in the current world model.")
				world = self.domain.getTriggers()[action.name](world, action.parameters, checked=False, verbose=self.verbose)
				if self.verbose:
					print 'result:'
					print world
				#world.graph.toXML('after_plan_step'+str(line)+".xml")

			if self.verbose: print 'Done executing actions. Let\'s see what we\'ve got (computing score and checking if the goal was achieved).'
			# Get result
			score, achieved, ignore = self.target(world.graph) # , symbolMapping
			## We store the result to check the plan
			self.valid = achieved
			if achieved:               # On the one hand, if we achieve the target world status, we will print all the  correct actions of the plan.
				if self.verbose: print 'GOAL ACHIEVED'
				self.achieved = True
				if self.verbose:
					for action in self.plan:
						print action
			else:                      # Otherwise, if we dont achieve the goal, we will print an error message.
				self.achieved = False
				if self.verbose: print 'Not achieved (didn\'t get to the goal)'

		# If we have thrown an exception (because a parameter of an action does not exist),
		# we handle part of the exception in this code.
		except WrongRuleExecution, e:
			if self.verbose: print 'Invalid rule execution', action
			if self.verbose: print 'Rule: ', e
			if self.verbose: print 'Line: ', line
			if self.verbose: print 'Not achieved'
			self.valid = False
			self.achieved = False
			if self.verbose: traceback.print_exc()
		except:
			self.valid = False
			self.achieved = False
			if self.verbose: print 'Not achieved (error)'
			if self.verbose: traceback.print_exc()
		return world.graph



"----------------------------------------------------------------------"
"----------------------------------------------------------------------"
## @brief This method prints the USER MODE of the AGGLPlanChecker class.
# " Usage
#   PROGRAM_NAME domain.[py/aggl] init.xml plan.plan target.[py/xml] [result.xml]
def printUsage():
	print 'Usage\n\t', sys.argv[0], ' domain.[py/aggl] init.xml plan.plan target.[py/xml]   [result.xml]'
	sys.exit(0)


"------------------------------------------------------------------------------------------"
"------------------------------------------------------------------------------------------"
# PRINCIPAL PROCEDURE --> MAIN PROGRAM
"------------------------------------------------------------------------------------------"
"------------------------------------------------------------------------------------------"
if __name__ == '__main__': # program domain problem result
	# We check the correct number of arguments
	if len(sys.argv)<5 or len(sys.argv)>6:
		printUsage()
	# If everything is correct, we save the arguments in local variables.
	domain = sys.argv[1] ## The grammar rules
	init   = sys.argv[2] ## The initial world status
	plan   = sys.argv[3] ## The plan genetared by the planner
	target = sys.argv[4] ## The target or goal world status.

	if len(sys.argv)==6:
		result = sys.argv[5] ## We save the name of the file where the result will be stored.
	else:
		result = ''

	agmData = None
	# If the grammar is stored in a .aggl file, we must generate the domain python file.
	if domain.endswith('.aggl'):
		## The domain python file is stored in agmData
		agmData = AGMFileDataParsing.fromFile(domain)
		# We save the python code in the domain.py file.
		agmData.generateAGGLPlannerCode("/tmp/domain.py", skipPassiveRules=True)
		domain = "/tmp/domain.py"
	# If the domain file doesnt finish with the correct extension, we will show an error message
	elif not domain.endswith('.py'):
		print "Domain extension must be 'py' or 'aggl'"
		printUsage()


	#
	# Generate the target module.
	# a) XML targets are not supported anymore to define targets
	# b) If an AGGT is provided it is converted to python code and then treated as if
	#    a python file was prodived
	# c) Python code is imported as a module and the CheckTarget function is used
	if target.endswith('.xml'):
		print 'XML targets are not supported anymore. Use AGGT files.'
		printUsage()
	# If the target file doesnt end with the cirrect extension, we will show an error message.
	elif target.endswith('.aggt'):
		temp = AGMFileDataParsing.targetFromFile(target)
		target = generateTarget_AGGT(agmData, temp)
	# If the target file doesnt end with the cirrect extension, we will show an error message.
	elif target.endswith('.py'):
		target = open(target, 'r').read()
	else:
		print "Target extension must be 'py' or 'aggt'"
		printUsage()
	m = imp.new_module('targetModule')
	exec target in m.__dict__
	target = m.CheckTarget

	# Generate a WorldStateHistory from the initital XML file
	init = WorldStateHistory([xmlModelParser.graphFromXMLFile(init), agmData.getInitiallyAwakeRules()])

	import tempfile
	with tempfile.NamedTemporaryFile() as temp:
		agmData.generateAGGLPlannerCode(temp.name, skipPassiveRules=True)
		domainRuleSet = imp.load_source('module__na_me', temp.name).RuleSet()
		p = AGGLPlanChecker(agmData, domainRuleSet, init, plan, target, dict(), True)
		ret = p.run()
	if result != '':
		ret.toXML(result)
