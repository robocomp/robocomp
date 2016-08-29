#!/usr/bin/env pypy

import sys, traceback
sys.path.append('/usr/local/share/agm/')

import xmlModelParser
from AGGL import *

from parseAGGL import AGMFileDataParsing
from agglplanner import *

if __name__ == '__main__': # program domain problem result
	if len(sys.argv)<4:
		print 'Usage\n\t', sys.argv[0], ' domain.aggl init.xml rulename [param1:value1 [param2:value2 [...]]]'
	else:
		agmData = AGMFileDataParsing.fromFile(sys.argv[1])

		params = dict()
		for i in xrange(4, len(sys.argv)):
			param = sys.argv[i].split(':')
			params[param[0]] = param[1]
		#params = eval(sys.argv[4])


		domainPath = "/tmp/domain.py"
		domain = imp.load_source('domain', domainPath).RuleSet()
		#             aggl   py-domain         init                                        target  indent  params             exclude  write-plan
		p = PyPlan(agmData, domainPath, sys.argv[2], domain.getHierarchicalTargets()[sys.argv[3]],     '', params,      [sys.argv[3]],       None)

