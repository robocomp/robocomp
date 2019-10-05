#!/usr/bin/env python
from pprint import pprint

from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine, delimitedList, Each
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, CaselessLiteral, CaselessKeyword, ParseBaseException
from collections import Counter
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
	ret = []
	c = Counter(aalist)
	keys = sorted(c)
	
	for k in keys:
		for cont in range(c[k]):
			if cont > 0:
				ret.append([k, str(cont)])
			else:
				ret.append([k, ''])
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
	def getCDSLParser():
		OBRACE,CBRACE,SEMI,OPAR,CPAR = map(Suppress, "{};()")
		QUOTE = Suppress(Word("\""))

		# keywords
		(IMPORT, COMMUNICATIONS, LANGUAGE, COMPONENT, CPP, CPP11, GUI, QWIDGET, QMAINWINDOW, QDIALOG, USEQt, QT, QT4, QT5, PYTHON, REQUIRES, IMPLEMENTS, SUBSCRIBESTO, PUBLISHES, OPTIONS, TRUE, FALSE,
		 INNERMODELVIEWER, STATEMACHINE) = map(CaselessKeyword, """
		import communications language component cpp cpp11 gui QWidget QMainWindow QDialog useQt Qt qt4 qt5
		python requires implements subscribesTo publishes options true false
		InnerModelViewer statemachine""".split())

		identifier = Word( alphas+"_", alphanums+"_" )

		# Imports
		idslImport  = Group(Suppress(IMPORT) - QUOTE + CharsNotIn("\";").setResultsName('idsl_path') - QUOTE + SEMI)
		idslImports = ZeroOrMore(idslImport).setResultsName("imports")

		commType = Optional(OPAR - (CaselessKeyword("ice") | CaselessKeyword("ros")).setResultsName("type") + CPAR)

		implementsList = Optional(IMPLEMENTS - delimitedList(Group(identifier.setResultsName("impIdentifier")+commType)).setResultsName("implements") + SEMI)

		requiresList = Optional(REQUIRES - delimitedList(Group(identifier.setResultsName("reqIdentifier")+commType)).setResultsName("requires") + SEMI)

		subscribesList = Optional(SUBSCRIBESTO - delimitedList(Group(identifier.setResultsName("subIdentifier")+commType)).setResultsName("subscribes") + SEMI)

		publishesList = Optional(PUBLISHES - delimitedList(Group(identifier.setResultsName("pubIdentifier")+commType)).setResultsName("publishes") + SEMI)

		communicationList = Group(implementsList & requiresList & subscribesList & publishesList).setResultsName("communications")
		communications = COMMUNICATIONS.suppress() - OBRACE + communicationList + CBRACE + SEMI


		# Language
		language_options = (CPP | CPP11 | PYTHON).setResultsName('language')
		language = LANGUAGE.suppress() - language_options - SEMI
		# Qtversion
		qtVersion = Group(Optional(USEQt.suppress() - (QT4|QT5).setResultsName('useQt') + SEMI))
		# InnerModelViewer
		innermodelviewer = Group(Optional(INNERMODELVIEWER.suppress() - (TRUE|FALSE) + SEMI))('innermodelviewer')
		# GUI
		gui_options = QWIDGET|QMAINWINDOW|QDIALOG
		gui = Group(Optional(GUI.suppress() - QT + OPAR - gui_options('gui_options') - CPAR + SEMI ))
		# additional options
		options = Group(Optional(OPTIONS.suppress() - identifier + ZeroOrMore(Suppress(Word(',')) + identifier) + SEMI))
		statemachine = Group(Optional(STATEMACHINE.suppress() - QUOTE + CharsNotIn("\";").setResultsName('machine_path') + QUOTE + SEMI))

		# Component definition
		componentContents = Group(communications - language + Optional(gui('gui')) + Optional(options('options')) + Optional(qtVersion) + Optional(innermodelviewer) + Optional(statemachine('statemachine'))).setResultsName("content")
		component = Group(COMPONENT.suppress() - identifier("name") + OBRACE + componentContents + CBRACE + SEMI).setResultsName("component")

		CDSL = idslImports - component


		return CDSL


	@staticmethod
	def fromString(inputText, verbose=False, includeDirectories=None):
		if includeDirectories == None:
			includeDirectories = []
		if verbose: print 'Verbose:', verbose
		text = nestedExpr("/*", "*/").suppress().transformString(inputText) 

		CDSL = CDSLParsing.getCDSLParser()
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

	# TODO: Check if we can use lru_cache decorator.
	@staticmethod
	def generateRecursiveImports(initial_idsls,  include_directories):

		new_idsls = []
		for idsl_path in initial_idsls:
			importedModule = None
			idsl_basename = os.path.basename(idsl_path)
			iD = include_directories + ['/opt/robocomp/interfaces/IDSLs/', os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
			try:
				for directory in iD:
					attempt = directory + '/' + idsl_basename
					# print 'Check', attempt
					if os.path.isfile(attempt):
						importedModule = IDSLParsing.fromFile(attempt)  # IDSLParsing.gimmeIDSL(attempt)
						break
			except:
				print 'Error reading IMPORT', idsl_basename
				traceback.print_exc()
				print 'Error reading IMPORT', idsl_basename
				os._exit(1)
			if importedModule == None:
				print 'Counldn\'t locate', idsl_basename
				os._exit(1)


			# if importedModule['imports'] have a # at the end an emtpy '' is generated
			idsl_imports = importedModule['imports'].split('#')
			# we remove all the '' ocurrences and existing imports
			aux_imports = []
			for i_import in idsl_imports:
				if i_import != '' and i_import not in initial_idsls:
					if communicationIsIce(i_import):
						aux_imports.append(i_import)
			idsl_imports = aux_imports
			if len(idsl_imports) > 0 and idsl_imports[0] != '':
				new_idsls += idsl_imports + CDSLParsing.generateRecursiveImports(idsl_imports,include_directories)

		return list(set(new_idsls))

	@staticmethod
	def component(tree, includeDirectories=None, start=''):
		component = {}
		# print 'parseCDSL.component', includeDirectories
		if includeDirectories == None:
			includeDirectories = []

		# Set options
		component['options'] = []
		try:
			for op in tree['component']['content']['options']:
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
		iD = includeDirectories + ['/opt/robocomp/interfaces/IDSLs/',
		                           os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
		for imp in sorted(imprts):
			import_basename = os.path.basename(imp['idsl_path'])
			component['imports'].append(import_basename)
		component['recursiveImports'] = CDSLParsing.generateRecursiveImports(component['imports'], includeDirectories)
		# Language
		component['language'] = tree['component']['content']['language']
# Statemachine
		component['statemachine'] = None
		try:
			statemachine = tree['component']['content']['statemachine']['machine_path']
			component['statemachine'] = statemachine
		except:
			pass
		
# qtVersion
		component['useQt'] = False
		try:
			component['useQt'] = tree['component']['content']['useQt']
			pass
		except:
			pass
		# innermodelviewer
		component['innermodelviewer'] = False
		try:
			component['innermodelviewer'] = 'innermodelviewer' in [ x.lower() for x in component['options'] ]
			pass
		except:
			pass
		# GUI
		component['gui'] = None
		try:
			uiT = tree['component']['content']['gui'][0]
			uiI = tree['component']['content']['gui']['gui_options']
			if uiT.lower() == 'qt' and uiI in ['QWidget', 'QMainWindow', 'QDialog' ]:
				component['gui'] = [ uiT, uiI ]
				pass
			else:
				print 'Wrong UI specification', tree['properties']['gui']
				sys.exit(1)
		except:
			# TODO: check exceptions and do something when accessing gui options fails.
			pass


		# Communications
		component['rosInterfaces']= []
		component['iceInterfaces']= []
		component['implements']   = []
		component['requires']     = []
		component['publishes']    = []
		component['subscribesTo'] = []
		component['usingROS'] = False
		####################
		com_types = ['implements', 'requires', 'publishes', 'subscribesTo']
		com_tree = tree['component']['content']['communications']
		for comm in com_types:
			if comm in com_tree:
				for interface in com_tree[comm]:
					component[comm].append(interface[0])
					if communicationIsIce(interface[0]):
						component['iceInterfaces'].append(interface[0])
					else:
						component['rosInterfaces'].append(interface[0])
						component['usingROS'] = True
		# Handle options for communications
		if isAGM1Agent(component):
			component['iceInterfaces'] += ['AGMCommonBehavior', 'AGMExecutive', 'AGMWorldModel']
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
	parser = CDSLParsing.getCDSLParser()
	result = parser.parseString("""
import "HandDetection.idsl";
import "CameraSimple.idsl";
import "RGBD.idsl";
import "CommonBehavior.idsl";
import "TvGames.idsl";
import "GetAprilTags.idsl";
import "TouchPoints.idsl";
import "AdminGame.idsl";
import "GameMetrics.idsl";

Component tvgames
{
	Communications
	{
		requires HandDetection (ros), CameraSimple (ice), RGBD, GetAprilTags;
        implements CommonBehavior, TvGames, AdminGame;
		publishes TouchPoints, GameMetrics;
	};
	language Python;
	gui Qt(QWidget);
	useQt Qt5;
	statemachine "gamestatemachine.smdsl";
};


	""")
	pprint(result.asDict())

	# component = CDSLParsing.fromFile(sys.argv[1])
	# pprint(component)


# 	component = CDSLParsing.getCDSLParser("""
# import "HandDetection.idsl";
# import "CameraSimple.idsl";
# import "RGBD.idsl";
# import "CommonBehavior.idsl";
# import "TvGames.idsl";
# import "GetAprilTags.idsl";
# import "TouchPoints.idsl";
# import "AdminGame.idsl";
# import "GameMetrics.idsl";
#
# Component tvgames
# {
# 	Communications
# 	{
# 		requires HandDetection (ros), CameraSimple (ice), RGBD, GetAprilTags;
#         implements CommonBehavior, TvGames, AdminGame;
# 		publishes TouchPoints, GameMetrics;
# 	};
# 	language Python;
# 	gui Qt(QWidget);
# 	useQt Qt5;
# 	statemachine "gamestatemachine.smdsl";
# };
#
#
# 	""")
# 	pprint(component)


	# with open(sys.argv[1]) as f:
	# 	result = parser.parseString(f.read(), )
	# 	pprint(result.asDict())
