#!/usr/bin/env python

from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, CaselessLiteral, CaselessKeyword, ParseBaseException 

import sys, traceback, os

debug = False
#debug = True

from parseIDSL import *
import rcExceptions

def getTypeFromModule(vtype, module):
	for t in module['types']:
		if t['name'] == vtype:
			return t['type']
	return None


def getKindFromPool(vtype, modulePool, debug=False):
	if debug: print vtype
	split = vtype.split("::")
	if debug: print split
	if len(split) > 1:
		vtype = split[1]
		mname = split[0]
		if debug: print 'SPLIT (' + vtype+'), (' + mname + ')'
		if mname in modulePool.modulePool:
			if debug: print 'dentro SPLIT (' + vtype+'), (' + mname + ')'
			r = getTypeFromModule(vtype, modulePool.modulePool[mname])
			if r != None: return r
		if mname.startswith("RoboComp"):
			if mname[8:] in modulePool.modulePool:
				r = getTypeFromModule(vtype, modulePool.modulePool[mname[8:]])
				if r != None: return r
	else:
		if debug: print 'no split'
		for module in modulePool.modulePool:
			if debug: print '  '+str(module)
			r = getTypeFromModule(vtype, modulePool.modulePool[module])
			if r != None: return r


def decoratorAndType_to_const_ampersand(decorator, vtype, modulePool, cpp11=False):
	ampersand = ' & '
	const = ' '

	if vtype in [ 'float', 'int', 'short', 'long', 'double' ]: # MAIN BASIC TYPES
		if decorator in [ 'out' ]: #out
			ampersand = ' &'
			const = ' '
		else:                      #read-only
			ampersand = ' '
			const = 'const '
	elif vtype in [ 'bool' ]:        # BOOL SEEM TO BE SPECIAL
		const = ' '
		if decorator in [ 'out' ]: # out
			ampersand = ' &'
		else:                      #read-only
			ampersand = ' '
	elif vtype in [ 'string' ]:      # STRINGS
		if decorator in [ 'out' ]: # out
			const = ' '
			ampersand = ' &'
		else:                      #read-only
			const = 'const '
			ampersand = ' &'
	else:                            # GENERIC, USED FOR USER-DEFINED DATA TYPES
		kind = getKindFromPool(vtype, modulePool)
		if kind == None:
			kind = getKindFromPool(vtype, modulePool, debug=True)
			raise Exception('error, unknown data structure, map or sequence '+vtype)
		else:
			if kind == 'enum':               # ENUM
				const = ' '
				if decorator in [ 'out' ]: # out
					ampersand = ' &'
				else:                      #read-only
					ampersand = ' '
			else:                            # THE REST
				if decorator in [ 'out' ]: # out
					ampersand = ' &'
					const = ' '
				else:                      # read-only
					if not cpp11:
						ampersand = ' &'
						const = 'const '
					else:
						ampersand = ''
						const = ''

	return const, ampersand



def getNameNumber(aalist):
	somelist = sorted(aalist)
	lastNum = 0
	ret = []
	for rqi, rq in enumerate(somelist):
		dup = False
		if rqi < len(somelist)-1:
			if str(rq) == str(somelist[rqi+1]):
				dup = True
		if rqi > 0:
			if str(rq) == str(somelist[rqi-1]):
				dup = True
		name = rq
		num = ''
		if dup:
			lastNum += 1
			num = str(lastNum)
			name = rq
		ret.append([name, num])
	return ret

class CDSLParsing:
	@staticmethod
	def fromFile(filename, verbose=False, includeIncludes=True, includeDirectories=None):
		# print 'fromFile', includeDirectories
		if includeDirectories == None:
			includeDirectories = []
		inputText = open(filename, 'r').read()
		try:
			# print 'fromFile2', includeDirectories
			ret = CDSLParsing.fromString(inputText, includeDirectories=includeDirectories)
		except:
			print 'Error reading', filename
			traceback.print_exc()
			print 'Error reading', filename
			sys.exit(1)
		ret['filename'] = filename
		return ret

	@staticmethod
	def fromString(inputText, verbose=False, includeDirectories=None):
		if includeDirectories == None:
			includeDirectories = []
		if verbose: print 'Verbose:', verbose
		text = nestedExpr("/*", "*/").suppress().transformString(inputText) 

		OBRACE,CBRACE,SEMI,OPAR,CPAR = map(Suppress, "{};()")
		QUOTE     					 = Suppress(Word("\""))

		# keywords
		(IMPORT, COMMUNICATIONS, LANGUAGE, COMPONENT, CPP, CPP11, GUI, USEQt, QT, QT4, QT5, PYTHON, REQUIRES, IMPLEMENTS, SUBSCRIBESTO, PUBLISHES, OPTIONS, TRUE, FALSE,
		 InnerModelViewer) = map(CaselessKeyword, """
		import communications language component cpp cpp11 gui useQt Qt qt4 qt5
		python requires implements subscribesTo publishes options true false
		InnerModelViewer""".split())

		identifier = Word( alphas+"_", alphanums+"_" )

		commIdentifier = Group(identifier('identifier') + Optional(OPAR + (CaselessKeyword("ice")|CaselessKeyword("ros")).setResultsName("type") + CPAR))

		# Imports
		idslImport  = Suppress(IMPORT) - QUOTE +  CharsNotIn("\";").setResultsName('path') - QUOTE + SEMI
		idslImports = ZeroOrMore(idslImport)
		
		# Communications
		implementsList = Group(IMPLEMENTS + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
		requiresList   = Group(REQUIRES + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
		subscribesList = Group(SUBSCRIBESTO + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
		publishesList  = Group(PUBLISHES + commIdentifier + ZeroOrMore(Suppress(Word(',')) + commIdentifier) + SEMI)
		communicationList = implementsList | requiresList | subscribesList | publishesList
		communications = Group( COMMUNICATIONS.suppress() + OBRACE + ZeroOrMore(communicationList) + CBRACE + SEMI)
		
		# Language
		language = Group(LANGUAGE.suppress() - (CPP | CPP11 | PYTHON) - SEMI)
		# Qtversion
		qtVersion = Group(Optional(USEQt.suppress() + (QT4|QT5) + SEMI))
		# InnerModelViewer
		innermodelviewer = Group(Optional(InnerModelViewer.suppress() + (TRUE|FALSE) + SEMI))
		# GUI
		gui = Group(Optional(GUI.suppress() - QT + OPAR - identifier - CPAR + SEMI ))
		# additional options
		options = Group(Optional(OPTIONS.suppress() + identifier + ZeroOrMore(Suppress(Word(',')) + identifier) + SEMI))

		# Component definition
		componentContents = communications('communications') + language('language') + Optional(gui('gui')) + Optional(options('options')) + Optional(qtVersion('useQt')) + Optional(innermodelviewer('innermodelviewer'))
		component = COMPONENT.suppress() + identifier("name") + OBRACE + componentContents("properties") + CBRACE + SEMI
		
		CDSL = idslImports("imports") - component("component")
		CDSL.ignore( cppStyleComment )
		try:
			tree = CDSL.parseString(text)
		except ParseBaseException as e:
			raise rcExceptions.ParseException(str(e), e.line, e.column)
		return CDSLParsing.component(tree, includeDirectories=includeDirectories)

	@staticmethod
	def printComponent(component, start=''):
		# Component name
		print 'Component', component['name']
		# Imports
		print '\tImports:'
		for imp in component['imports']:
			print '\t\t', imp
		# Language
		print '\tLanguage:'
		print '\t\t', component['language']
		# GUI
		print '\tGUI:'
		print '\t\t', component['gui']
		# Communications
		print '\tCommunications:'
		print '\t\tImplements', component['implements']
		print '\t\tRequires', component['requires']
		print '\t\tPublishes', component['publishes']
		print '\t\tSubscribes', component['subscribesTo']

	@staticmethod
	def component(tree, includeDirectories=None, start=''):
		component = {}
		# print 'parseCDSL.component', includeDirectories
		if includeDirectories == None:
			includeDirectories = []

		# Set options
		component['options'] = []
		try:
			for op in tree['properties']['options']:
				component['options'].append(op.lower())
		except:
			traceback.print_exc()

		# Component name
		component['name'] = tree['component']['name']
		# Imports
		component['imports'] = []
		component['recursiveImports'] = []
		try:
			imprts = tree['imports']
		except:
			tree['imports'] = []
			imprts = []
		if isAGM1Agent(component):
			imprts = ['AGMExecutive.idsl', 'AGMCommonBehavior.idsl', 'AGMWorldModel.idsl']
			for i in tree['imports']:
				if not i in imprts:
					imprts.append(i)
		if isAGM2Agent(component):
			imprts = ['AGM2.idsl']
			for i in tree['imports']:
				if not i in imprts:
					imprts.append(i)

		for imp in sorted(imprts):
			component['imports'].append(imp)
			importedModule = None
			try:
				iD = includeDirectories + ['/opt/robocomp/interfaces/IDSLs/', os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
				# print 'iD', iD
				for directory in iD:
					attempt = directory+'/'+imp
					# print 'Check', attempt
					if os.path.isfile(attempt):
						importedModule = IDSLParsing.fromFile(attempt) # IDSLParsing.gimmeIDSL(attempt)

			except:
				print 'Error reading IMPORT', imp
				traceback.print_exc()
				print 'Error reading IMPORT', imp
				os._exit(1)
			if importedModule == None:
				print 'Counldn\'t locate', imp
				os._exit(1)
			# recursiveImports holds the necessary imports
			importable = False
			for interf in importedModule['interfaces']:
				for comm in tree['properties']['communications']:
					for interface in comm[1:]:
						if communicationIsIce(interface):
							if interf['name'] == interface[0]:
								importable = True
			if importable:
				component['recursiveImports'] += [attempt]
				component['recursiveImports'] += [x for x in importedModule['imports'].split('#') if len(x)>0]

		# Language
		component['language'] = tree['properties']['language'][0]
		# qtVersion
		component['useQt'] = 'none'
		try:
			component['useQt'] = tree['properties']['useQt'][0]
			pass
		except:
			pass
		# innermodelviewer
		component['innermodelviewer'] = 'false'
		try:
			component['innermodelviewer'] = 'innermodelviewer' in [ x.lower() for x in component['options'] ]
			pass
		except:
			pass
		# GUI
		component['gui'] = 'none'
		try:
			uiT = tree['properties']['gui'][0]
			uiI = tree['properties']['gui'][1]
			if uiT.lower() == 'qt' and uiI in ['QWidget', 'QMainWindow', 'QDialog' ]:
				component['gui'] = [ uiT, uiI ]
				pass
			else:
				print 'Wrong UI specification', tree['properties']['gui']
				sys.exit(1)
		except:
			pass


		# Communications
		component['rosInterfaces']= []
		component['iceInterfaces']= []
		component['implements']   = []
		component['requires']     = []
		component['publishes']    = []
		component['subscribesTo'] = []
		component['usingROS'] = "None"
		####################
		com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
		communications = sorted(tree['properties']['communications'], key=lambda x: x[0])
		for comm in communications:
			if comm[0] in com_types:
				comm_type = comm[0]
				interfaces = sorted(comm[1:], key=lambda x: x[0])
				for interface in interfaces:
					component[comm_type].append(interface)
					if communicationIsIce(interface):
						component['iceInterfaces'].append(interface[0])
					else:
						component['rosInterfaces'].append(interface[0])
						component['usingROS'] = True
		#####################
		# for comm in tree['properties']['communications']:
		# 	if comm[0] == 'implements':
		# 		for interface in comm[1:]:
		# 			component['implements'].append(interface)
		# 			if communicationIsIce(interface):
		# 				component['iceInterfaces'].append(interface[0])
		# 			else:
		# 				component['rosInterfaces'].append(interface[0])
		# 				component['usingROS'] = True
		# 	if comm[0] == 'requires':
		# 		for interface in comm[1:]:
		# 			component['requires'].append(interface)
		# 			if communicationIsIce(interface):
		# 				component['iceInterfaces'].append(interface[0])
		# 			else:
		# 				component['rosInterfaces'].append(interface[0])
		# 				component['usingROS'] = True
		# 	if comm[0] == 'publishes':
		# 		for interface in comm[1:]:
		# 			component['publishes'].append(interface)
		# 			if communicationIsIce(interface):
		# 				component['iceInterfaces'].append(interface[0])
		# 			else:
		# 				component['rosInterfaces'].append(interface[0])
		# 				component['usingROS'] = True
		# 	if comm[0] == 'subscribesTo':
		# 		for interface in comm[1:]:
		# 			component['subscribesTo'].append(interface)
		# 			if communicationIsIce(interface):
		# 				component['iceInterfaces'].append(interface[0])
		# 			else:
		# 				component['rosInterfaces'].append(interface[0])
		# 				component['usingROS'] = True
		# Handle options for communications
		if isAGM1Agent(component):
			if not 'AGMCommonBehavior' in component['implements']:
				component['implements'] =   ['AGMCommonBehavior'] + component['implements']
			if not 'AGMExecutive' in component['requires']:
				component['requires'] =   ['AGMExecutive'] + component['requires']
			if not 'AGMExecutiveTopic' in component['subscribesTo']:
				component['subscribesTo'] = ['AGMExecutiveTopic'] + component['subscribesTo']
		if isAGM2Agent(component):
			if isAGM2AgentROS(component):
				component['usingROS'] = True
				agm2agent_requires = [['AGMDSRService','ros']]
				agm2agent_subscribesTo = [['AGMExecutiveTopic','ros'], ['AGMDSRTopic','ros']]
				if not 'AGMDSRService'     in component['rosInterfaces']: component['rosInterfaces'].append('AGMDSRService')
				if not 'AGMDSRTopic'       in component['rosInterfaces']: component['rosInterfaces'].append('AGMDSRTopic')
				if not 'AGMExecutiveTopic' in component['rosInterfaces']: component['rosInterfaces'].append('AGMExecutiveTopic')
			else:
				agm2agent_requires = [['AGMDSRService','ice']]
				agm2agent_subscribesTo = [['AGMExecutiveTopic','ice'], ['AGMDSRTopic','ice']]
				if not 'AGMDSRService'     in component['iceInterfaces']: component['iceInterfaces'].append('AGMDSRService')
				if not 'AGMDSRTopic'       in component['iceInterfaces']: component['iceInterfaces'].append('AGMDSRTopic')
				if not 'AGMExecutiveTopic' in component['iceInterfaces']: component['iceInterfaces'].append('AGMExecutiveTopic')

			# AGM2 agents REQUIRES
			for agm2agent_req in agm2agent_requires:
				if not agm2agent_req in component['requires']:
					component['requires'] =   [agm2agent_req] + component['requires']
			# AGM2 agents SUBSCRIBES
			for agm2agent_sub in agm2agent_subscribesTo:
				if not agm2agent_sub in component['subscribesTo']:
					component['subscribesTo'] = [agm2agent_sub] + component['subscribesTo']
		return component

def communicationIsIce(sb):
	isIce = True
	if len(sb) == 2:
		if sb[1] == 'ros'.lower():
			isIce = False
		elif sb[1] != 'ice'.lower() :
			print('Only ICE and ROS are supported')
			sys.exit(-1)
	return isIce


def isAGM1Agent(component):
	options = component['options']
	return 'agmagent' in [ x.lower() for x in options]

def isAGM2Agent(component):
	valid = ['agm2agent', 'agm2agentros', 'agm2agentice']
	options = component['options']
	for v in valid:
		if v.lower() in options:
			return True
	return False

def isAGM2AgentROS(component):
	valid = ['agm2agentROS']
	options = component['options']
	for v in valid:
		if v.lower() in options:
			return True
	return False

if __name__ == '__main__':
	CDSLParsing.fromFile(sys.argv[1])
