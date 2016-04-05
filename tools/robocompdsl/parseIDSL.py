#!/usr/bin/env python

from pyparsing import Word, alphas, alphanums, nums, OneOrMore, CharsNotIn, Literal, Combine
from pyparsing import cppStyleComment, Optional, Suppress, ZeroOrMore, Group, StringEnd, srange
from pyparsing import nestedExpr, ParseException

import sys, traceback, os



class IDSLParsing:
	@staticmethod
	def fromFile(filename):
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
	def fromString(inputText):
		text = nestedExpr("/*", "*/").suppress().transformString(inputText)

		semicolon = Suppress(Word(";"))
		quote     = Suppress(Word("\""))
		op        = Suppress(Word("{"))
		cl        = Suppress(Word("}"))
		opp       = Suppress(Word("("))
		clp       = Suppress(Word(")"))
		lt       = Suppress(Word("<"))
		gt       = Suppress(Word(">"))
		identifier        = Word(alphas+"_",alphanums+"_")
		typeIdentifier    = Word(alphas+"_",alphanums+"_:")
		structIdentifer   = Group(typeIdentifier.setResultsName('type') + identifier.setResultsName('identifier') + semicolon)
		structIdentifers  = Group(structIdentifer + OneOrMore(structIdentifer))

		## Imports
		idslImport  = Suppress(Word("import")) + quote +  CharsNotIn("\";").setResultsName('path') + quote + semicolon
		idslImports = ZeroOrMore(idslImport)
		
		structDef     = Word("struct").setResultsName('type') + identifier.setResultsName('name') + op + structIdentifers.setResultsName("structIdentifiers") + cl + semicolon
		dictionaryDef = Word("dictionary").setResultsName('type') + lt + CharsNotIn("<>") + gt + identifier.setResultsName('name') + semicolon
		sequenceDef   = Word("sequence").setResultsName('type')   + lt + typeIdentifier.setResultsName('typeSequence') + gt + identifier.setResultsName('name') + semicolon
		enumDef       = Word("enum").setResultsName('type')       + identifier.setResultsName('name') + op + CharsNotIn("{}") + cl + semicolon
		exceptionDef  = Word("exception").setResultsName('type')  + identifier.setResultsName('name') + op + CharsNotIn("{}") + cl + semicolon

		raiseDef       = Suppress(Word("throws")) + typeIdentifier + ZeroOrMore( Literal(',') + typeIdentifier )
		decoratorDef    = Literal('idempotent') | Literal('out')
		retValDef       = typeIdentifier.setResultsName('ret')

		firstParam    = Group( Optional(decoratorDef.setResultsName('decorator')) + typeIdentifier.setResultsName('type') + identifier.setResultsName('name'))
		nextParam     = Suppress(Word(',')) + firstParam
		params        = firstParam + ZeroOrMore(nextParam)


		remoteMethodDef  = Group(Optional(decoratorDef) + retValDef + typeIdentifier.setResultsName('name') + opp + Optional(          params).setResultsName('params') + clp + Optional(raiseDef) + semicolon )
		interfaceDef    = Word("interface").setResultsName('type')  + typeIdentifier.setResultsName('name') + op + Group(ZeroOrMore(remoteMethodDef)) + cl + semicolon

		moduleContent = Group(structDef | enumDef | exceptionDef | dictionaryDef | sequenceDef | interfaceDef)
		module = Suppress(Word("module")) + identifier.setResultsName("name") + op + ZeroOrMore(moduleContent).setResultsName("contents") + cl + semicolon

		IDSL = idslImports.setResultsName("imports") + module.setResultsName("module")
		IDSL.ignore( cppStyleComment )
		tree = IDSL.parseString(text)
		return tree

if __name__ == '__main__':
	sequencesList = []
	idsl = IDSLParsing.fromFile(sys.argv[1])
	for imp in idsl['module']['contents']:
		if imp['type'] == 'sequence':
			sequencesList.append(imp)
		if imp['type'] == 'struct':
			for structVar in imp['structIdentifiers']:
				wasSequence = False
				for sequences in sequencesList:
					if structVar[0] == sequences['name']:
						print sequences['typeSequence']
						wasSequence = True
						break
				if not wasSequence:
					print "not wasSequence"