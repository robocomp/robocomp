#!/usr/bin/env python

from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, ParseException

import sys, traceback, os

debug = False
#debug = True



class IDSLParsing:
	"""generic class for parsing idsl"""
	@staticmethod
<<<<<<< HEAD
	def fromFileIDSL(filename):
		inputText = open(filename, 'r').read()
		try:
			ret = IDSLParsing.fromString(inputText)
		except ParseException, p:
			print 'Error reading IDSL', filename
			traceback.print_exc()
			print 'Error reading IDSL', filename
			print p.markInputline()
			os._exit(1)
		ret['filename'] = filename
		return ret

	@staticmethod
	def fromFile(filename):
=======
	def fromFile(filename, verbose=False, includeIncludes=True):
		# Open input file
		#inputText = "\n".join([line for line in open(filename, 'r').read().split("\n") if not line.lstrip(" \t").startswith('//')])
>>>>>>> master
		inputText = open(filename, 'r').read()
		try:
			ret = IDSLParsing.fromString(inputText)
		except ParseException, p:
			print 'Error reading IDSL', filename
			traceback.print_exc()
			print 'Error reading IDSL', filename
			print p.markInputline()
			os._exit(1)
		ret['filename'] = filename
		return ret
	@staticmethod
	def fromString(inputText, verbose=False):
		if verbose: print 'Verbose:', verbose
		text = nestedExpr("/*", "*/").suppress().transformString(inputText)

		semicolon = Suppress(Word(";"))
		quote     = Suppress(Word("\""))
		op        = Suppress(Word("{"))
		cl        = Suppress(Word("}"))
		opp       = Suppress(Word("("))
		clp       = Suppress(Word(")"))
<<<<<<< HEAD
		lt        = Suppress(Word("<"))
		gt        = Suppress(Word(">"))
		eq        = Suppress(Word("="))
		identifier        = Word(alphas+"_",alphanums+"_")
		typeIdentifier    = Word(alphas+"_",alphanums+"_:")
		structIdentifer   = Group(typeIdentifier.setResultsName('type') + identifier.setResultsName('identifier') + Optional(eq) + Optional(CharsNotIn(";").setResultsName('defaultValue')) + semicolon)
		structIdentifers  = Group(OneOrMore(structIdentifer))
=======
		lt       = Suppress(Word("<"))
		gt       = Suppress(Word(">"))
		identifier     = Word(alphas+"_",alphanums+"_")
		typeIdentifier = Word(alphas+"_",alphanums+"_:")

>>>>>>> master

		## Imports
		idslImport  = Suppress(Word("import")) + quote +  CharsNotIn("\";").setResultsName('path') + quote + semicolon
		idslImports = ZeroOrMore(idslImport)

<<<<<<< HEAD
		structDef     = Word("struct").setResultsName('type') + identifier.setResultsName('name') + op + structIdentifers.setResultsName("structIdentifiers") + cl + semicolon
		dictionaryDef = Word("dictionary").setResultsName('type') + lt + CharsNotIn("<>").setResultsName('content') + gt + identifier.setResultsName('name') + semicolon
		sequenceDef   = Word("sequence").setResultsName('type')   + lt + typeIdentifier.setResultsName('typeSequence') + gt + identifier.setResultsName('name') + semicolon
		enumDef       = Word("enum").setResultsName('type')       + identifier.setResultsName('name') + op + CharsNotIn("{}").setResultsName('content') + cl + semicolon
		exceptionDef  = Word("exception").setResultsName('type')  + identifier.setResultsName('name') + op + CharsNotIn("{}").setResultsName('content') + cl + semicolon
=======

		dictionaryDef = Word("dictionary") + lt + CharsNotIn("<>;") + gt + identifier.setResultsName('name') + semicolon
		sequenceDef   = Word("sequence")   + lt + CharsNotIn("<>;") + gt + identifier.setResultsName('name') + semicolon
		enumDef       = Word("enum")       + identifier.setResultsName('name') + op + CharsNotIn("{}") + cl + semicolon
		structDef     = Word("struct")     + identifier.setResultsName('name') + op + CharsNotIn("{}") + cl + semicolon
		exceptionDef  = Word("exception")  + identifier.setResultsName('name') + op + CharsNotIn("{}") + cl + semicolon
>>>>>>> master

		raiseDef       = Suppress(Word("throws")) + typeIdentifier + ZeroOrMore( Literal(',') + typeIdentifier )
		decoratorDef    = Literal('idempotent') | Literal('out')
		retValDef       = typeIdentifier.setResultsName('ret')

		firstParam    = Group( Optional(decoratorDef.setResultsName('decorator')) + typeIdentifier.setResultsName('type') + identifier.setResultsName('name'))
		nextParam     = Suppress(Word(',')) + firstParam
		params        = firstParam + ZeroOrMore(nextParam)


<<<<<<< HEAD
		remoteMethodDef  = Group(Optional(decoratorDef.setResultsName('decorator')) + retValDef.setResultsName('ret') + typeIdentifier.setResultsName('name') + opp + Optional(          params).setResultsName('params') + clp + Optional(raiseDef.setResultsName('raise')) + semicolon )
		interfaceDef    = Word('interface').setResultsName('type')  + typeIdentifier.setResultsName('name') + op + Group(ZeroOrMore(remoteMethodDef)).setResultsName('methods') + cl + semicolon
=======
		remoteMethodDef  = Group(Optional(decoratorDef) + retValDef + typeIdentifier.setResultsName('name') + opp + Optional(          params).setResultsName('params') + clp + Optional(raiseDef) + semicolon )
		interfaceDef    = Word("interface")  + typeIdentifier.setResultsName('name') + op + Group(ZeroOrMore(remoteMethodDef)) + cl + semicolon
>>>>>>> master

		moduleContent = Group(structDef | enumDef | exceptionDef | dictionaryDef | sequenceDef | interfaceDef)
		module = Suppress(Word("module")) + identifier.setResultsName("name") + op + ZeroOrMore(moduleContent).setResultsName("contents") + cl + semicolon

		IDSL = idslImports.setResultsName("imports") + module.setResultsName("module")
		IDSL.ignore( cppStyleComment )
		tree = IDSL.parseString(text)
		return IDSLParsing.module(tree)

	@staticmethod
	def gimmeIDSL(name, files='', includeDirectories=None):
		pathList = []
		if includeDirectories!= None:
			pathList += [x for x in includeDirectories]
		fileList = []
		for p in [f for f in files.split('#') if len(f)>0]:
			if p.startswith("-I"):
				pathList.append(p[2:])
			else:
				fileList.append(p)
		pathList.append('/home/robocomp/robocomp/interfaces/IDSLs/')
		pathList.append('/opt/robocomp/interfaces/IDSLs/')
		filename = name.split('.')[0]
		for p in pathList:
			try:
				path = p+'/'+name
				return IDSLParsing.fromFile(path)
			except IOError, e:
				pass
		print 'Couldn\'t locate ', name
		sys.exit(-1)
	@staticmethod
	def gimmeIDSLStruct(name, files='', includeDirectories=None):
		pathList = []
		if includeDirectories!= None:
			pathList += [x for x in includeDirectories]
		fileList = []
		for p in [f for f in files.split('#') if len(f)>0]:
			if p.startswith("-I"):
				pathList.append(p[2:])
			else:
				fileList.append(p)
		pathList.append('/home/robocomp/robocomp/interfaces/IDSLs/')
		pathList.append('/opt/robocomp/interfaces/IDSLs/')
		filename = name.split('.')[0]
		for p in pathList:
			try:
				path = p+'/'+name
				return IDSLParsing.fromFileIDSL(path)
			except IOError, e:
				pass
		print 'Couldn\'t locate ', name
		sys.exit(-1)

	@staticmethod
	def module(tree, start=''):
		module = {}

		#module name
		module['name'] = tree['module']['name']

		module['imports'] = ''
		if 'imports' in tree:
			#print module['name'], tree['imports']
			for imp in tree['imports']:
				#print 'proc', imp
				#print 'has', IDSLParsing.gimmeIDSL(imp)['imports']
				#print ''
				module['imports'] += imp + '#' + IDSLParsing.gimmeIDSL(imp)['imports']
		# INTERFACES DEFINED IN THE MODULE
		module['interfaces'] = []
		for contentDef in tree['module']['contents']:
			if contentDef[0] == 'interface':
				interface = { 'name':contentDef[1], 'methods':{}}
				for method in contentDef[2]:
					interface['methods'][method['name']] = {}

					interface['methods'][method['name']]['name'] = method['name']
					try:
						interface['methods'][method['name']]['decorator'] = method['decorator']
					except:
						interface['methods'][method['name']]['decorator'] = ''

					interface['methods'][method['name']]['return'] = method['ret']

					params = []
					try:
						for p in method['params']:
							try:
								params.append( { 'decorator':p['decorator'], 'type':p['type'], 'name':p['name'] } )
							except:
								params.append( { 'decorator':'none',         'type':p['type'], 'name':p['name'] } )
					except:
						pass
					interface['methods'][method['name']]['params'] = params

					try:
						interface['methods'][method['name']]['throws'] = method['decorator']
					except:
						interface['methods'][method['name']]['throws'] = 'nothing'
				module['interfaces'].append(interface)
		# TYPES DEFINED IN THE MODULE
		module['types'] = []
		#print '---\n---\nPARSE IDSL TYPES'
		for contentDef in tree['module']['contents']:
			#print contentDef[0]
			if contentDef[0] in [ 'enum', 'struct', 'exception' ]:
				typedef = { 'name':contentDef[1], 'type':contentDef[0]}
				#print typedef
				module['types'].append(typedef)
			elif contentDef[0] in [ 'sequence', 'dictionary' ]:
				typedef = { 'name':contentDef[-1], 'type':contentDef[0]}
				#print typedef
				module['types'].append(typedef)
			elif contentDef[0] in ['interface']:
				pass
			else:
				print 'Unknown module content', contentDef
		return module

	@staticmethod
	def printModule(module, start=''):
		print 'MODULE', module['name']+':'
		print ' ', 'INTERFACES:'
		for interface in module['interfaces']:
			print '   ', interface['name']
			for mname in interface['methods']:
				method = interface['methods'][mname]
				print '     ', method['name']
				print '        decorator', method['decorator']
				print '        return', method['return']
				print '        params'
				for p in method['params']:
					print '         ', '<', p['decorator'], '>  <', p['type'], '>  <', p['name'], '>'






rosTypes = ('int8','int16','int32','int64','float8','float16','float32','float64','byte','bool','string','time','empty')

class IDSLPool:
<<<<<<< HEAD
	def __init__(self, files, iD):
=======
	"""Creates a pool of parsed idsl objects from list of files"""
	def __init__(self, files):
>>>>>>> master
		self.modulePool = {}
		includeDirectories = iD + ['/opt/robocomp/interfaces/IDSLs/', os.path.expanduser('~/robocomp/interfaces/IDSLs/')]
		self.includeInPool(files, self.modulePool, includeDirectories)
		self.rosTypes = rosTypes
	def getRosTypes(self):
		return self.rosTypes
	def includeInPool(self, files, modulePool, includeDirectories):
		fileList = []
		for p in [f for f in files.split('#') if len(f)>0]:
			if p.startswith("-I"):
				pass
			else:
				fileList.append(p)
		for f in fileList:
			filename = f.split('.')[0]
			if not filename in modulePool:
				for p in includeDirectories:
					try:
						path = p+'/'+f
						module = IDSLParsing.fromFile(path)
						modulePool[filename] = module
						self.includeInPool(module['imports'], modulePool, includeDirectories)
						break
					except IOError, e:
						pass
				if not filename in self.modulePool:
					print 'Couldn\'t locate ', f
					sys.exit(-1)
<<<<<<< HEAD
	def IDSLsModule(self, module):
		for filename in self.modulePool.keys():
			if self.modulePool[filename] == module:
				return '/opt/robocomp/interfaces/IDSLs/'+filename+'.idsl'
=======
>>>>>>> master

	def moduleProviding(self, interface):
		for module in self.modulePool:
			for m in self.modulePool[module]['interfaces']:
				if m['name'] == interface:
					return self.modulePool[module]
		return None

<<<<<<< HEAD
	def interfaces(self):
		interfaces = []
		for module in self.modulePool:
			for m in self.modulePool[module]['interfaces']:
				interfaces.append(m['name']) 
		return interfaces

	def rosImports(self):
		includesList = []
		for module in self.modulePool:
			for m in self.modulePool[module]['structs']:
				includesList.append(m['name'].split('/')[0]+"ROS/"+m['name'].split('/')[1])
			for m in self.modulePool[module]['sequences']:
				includesList.append(m['name'].split('/')[0]+"ROS/"+m['name'].split('/')[1])
			stdIncludes = {}
			for interface in self.modulePool[module]['interfaces']:
				for mname in interface['methods']:
					method = interface['methods'][mname]
					for p in method['params']:
						if p['type'] in ('int','float'):
							m = "std_msgs/"+p['type'].capitalize()+"32"
							stdIncludes[p['type']] = m
						elif p['type'] in ('uint8','uint16','uint32','uint64'):
							m = "std_msgs/UInt"+p['type'].split('t')[1]
						elif p['type'] in self.rosTypes:
							m = "std_msgs/"+p['type'].capitalize()
							stdIncludes[p['type']] = m
			for std in stdIncludes.values():
				includesList.append(std)
		return includesList

	def rosModulesImports(self):
		modulesList = []
		for module in self.modulePool:
			for m in self.modulePool[module]['simpleStructs']:
				modulesList.append(m)
			for m in self.modulePool[module]['simpleSequences']:
				modulesList.append(m)
		return modulesList
=======

>>>>>>> master


if __name__ == '__main__':
	idsl = IDSLParsing.fromFile(sys.argv[1])
<<<<<<< HEAD
	for imp in idsl['imports']:
		print imp
=======
	IDSLParsing.printModule(idsl)
>>>>>>> master
