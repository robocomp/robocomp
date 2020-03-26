#!/usr/bin/python3
# -*- coding: utf-8 -*-
[[[cog

import sys
sys.path.append('/opt/robocomp/python')

import cog
def A():
	cog.out('<@@<')
def Z():
	cog.out('>@@>')
def TAB():
	cog.out('<TABHERE>')

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import get_name_number, gimmeIDSL, communication_is_ice, IDSLPool

includeDirectories = theIDSLPaths.split('#')
component = DSLFactory().from_file(theCDSL, include_directories=includeDirectories)
sm = DSLFactory().from_file(component['statemachine'])
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)


pool = IDSLPool(theIDSLs, includeDirectories)

def replaceTypeCPP2Python(t):
	t = t.replace('::','.')
	t = t.replace('string', 'str')
	return t

]]]
[[[end]]]
#
# Copyright (C)
[[[cog
A()
import datetime
cog.out(' '+str(datetime.date.today().year))
Z()
]]]
[[[end]]]
 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
[[[cog
if sm is None:
	cog.outl("<TABHERE><TABHERE>self.timer.timeout.connect(self.compute)")
]]]
[[[end]]]
		self.Period = 2000
		self.timer.start(self.Period)

[[[cog
if sm is not None:
	cog.outl("<TABHERE><TABHERE>self." + sm['machine']['name'] + ".start()")
	if sm['machine']['default']:
		cog.outl("<TABHERE><TABHERE>self.destroyed.connect(self.t_compute_to_finalize)")
]]]
[[[end]]]

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True

[[[cog
if (sm is not None and sm['machine']['default'] is True) or component['statemachine'] is None:
	cog.outl("<TABHERE>@QtCore.Slot()")
	cog.outl("<TABHERE>def compute(self):")
	cog.outl("<TABHERE><TABHERE>print('SpecificWorker.compute...')")
	cog.outl("<TABHERE><TABHERE>#computeCODE")
	cog.outl("<TABHERE><TABHERE>#try:")
	cog.outl("<TABHERE><TABHERE>#<TABHERE>self.differentialrobot_proxy.setSpeedBase(100, 0)")
	cog.outl("<TABHERE><TABHERE>#except Ice.Exception as e:")
	cog.outl("<TABHERE><TABHERE>#<TABHERE>traceback.print_exc()")
	cog.outl("<TABHERE><TABHERE>#<TABHERE>print(e)")
	cog.outl("")
	cog.outl("<TABHERE><TABHERE># The API of python-innermodel is not exactly the same as the C++ version")
	cog.outl("<TABHERE><TABHERE># self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)")
	cog.outl("<TABHERE><TABHERE># z = librobocomp_qmat.QVec(3,0)")
	cog.outl("<TABHERE><TABHERE># r = self.innermodel.transform('rgbd', z, 'laser')")
	cog.outl("<TABHERE><TABHERE># r.printvector('d')")
	cog.outl("<TABHERE><TABHERE># print(r[0], r[1], r[2])")
	cog.outl("")
	cog.outl("<TABHERE><TABHERE>return True")
]]]
[[[end]]]

[[[cog
if sm is not None:
	codVirtuals = ""

	# Generate code for the methods of the StateMachine.
	if sm['machine']['contents']['initialstate'] is not None:

        #TODO: code to
        # if sm['machine']['contents']['transitions'] is not None:
        #     for transi in sm['machine']['contents']['transitions']:
        #             if sm['machine']['contents']['initialstate'] == trasi["src"]
        #                 codsignals += "<TABHERE>#<TABHERE>%s >>> %s" % ( transi['src'], sm['machine']['contents']['initialstate'])
        # if sm['machine']['contents']['transitions'] is not None:
        #     for transi in sm['machine']['contents']['transitions']:
        #         for dest in transi['dests']:
        #             if  sm['machine']['contents']['initialstate'] == dest
        #                 codsignals += "<TABHERE>#<TABHERE>%s <<< %s"%(sm['machine']['contents']['initialstate'], transi['src'])

		if sm['machine']['default']:
			codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + sm['machine']['contents']['initialstate'] + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + sm['machine']['contents']['initialstate'] + "(self):\n<TABHERE><TABHERE>print(\"Entered state " + sm['machine']['contents']['initialstate'] + "\")\n<TABHERE><TABHERE>self.t_initialize_to_compute.emit()\n<TABHERE><TABHERE>pass\n\n"
		else:
			codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + sm['machine']['contents']['initialstate'] + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + sm['machine']['contents']['initialstate'] + "(self):\n<TABHERE><TABHERE>print(\"Entered state " + sm['machine']['contents']['initialstate'] + "\")\n<TABHERE><TABHERE>pass\n\n"

	if sm['machine']['contents']['states'] is not None:
		for state in sorted(sm['machine']['contents']['states']):
			if sm['machine']['default']:
				if state == 'compute':
					codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + state + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + state + "(self):\n<TABHERE><TABHERE>print(\"Entered state " + state + "\")\n<TABHERE><TABHERE>self.compute()\n<TABHERE><TABHERE>pass\n\n"
			else:
				codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + state + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + state + "(self):\n<TABHERE><TABHERE>print(\"Entered state " + state + "\")\n<TABHERE><TABHERE>pass\n\n"

	if sm['machine']['contents']['finalstate'] is not None:
		codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + sm['machine']['contents']['finalstate'] + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + sm['machine']['contents']['finalstate'] + "(self):\n<TABHERE><TABHERE>print(\"Entered state "+sm['machine']['contents']['finalstate']+"\")\n<TABHERE><TABHERE>pass\n\n"

	# Generate code for the methods of the StateMachine transitions for substates.
	if sm['substates'] is not None:
		for substates in sm['substates']:
            #TODO: Add commented header with the parent of this methods.
			if substates['contents']['initialstate'] is not None:
				codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + substates['contents']['initialstate'] + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + substates['contents']['initialstate'] + "(self):\n<TABHERE><TABHERE>print(\"Entered state " + substates['contents']['initialstate'] + "\")\n<TABHERE><TABHERE>pass\n\n"
			if substates['contents']['states'] is not None:
				for state in sorted(substates['contents']['states']):
					codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + state + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + state + "(self):\n<TABHERE><TABHERE>print(\"Entered state "+state+"\")\n<TABHERE><TABHERE>pass\n\n"
			if substates['contents']['finalstate'] is not None:
				codVirtuals += "<TABHERE>#\n<TABHERE># sm_" + substates['contents']['finalstate'] + "\n<TABHERE>#\n<TABHERE>@QtCore.Slot()\n<TABHERE>def sm_" + substates['contents']['finalstate'] + "(self):\n<TABHERE><TABHERE>print(\"Entered state "+substates['contents']['finalstate']+"\")\n<TABHERE><TABHERE>pass\n\n"

	cog.outl("# =============== Slots methods for State Machine ===================")
	cog.outl("# ===================================================================")
	cog.outl(codVirtuals)
	cog.outl("# =================================================================")
	cog.outl("# =================================================================\n")
]]]
[[[end]]]
[[[cog
lst = []
try:
	lst += component['subscribesTo']
except:
	pass
for imp in lst:
	if type(imp) == str:
		im = imp
	else:
		im = imp[0]
	module = pool.moduleProviding(im)
	for interface in module['interfaces']:
		if interface['name'] == im:
			for mname in interface['methods']:
				method = interface['methods'][mname]
				outValues = []
				if method['return'] != 'void':
					outValues.append([method['return'], 'ret'])
				paramStrA = ''
				for p in method['params']:
					if p['decorator'] == 'out':
						outValues.append([p['type'], p['name']])
					else:
						paramStrA += ', ' +  p['name']
				cog.outl('')
				cog.outl('<TABHERE>#')
				cog.outl('<TABHERE># ' + 'SUBSCRIPTION to '+ method['name'] +' method from ' + interface['name'] + ' interface')
				cog.outl('<TABHERE>#')
				if not communication_is_ice(imp):
					cog.outl('<TABHERE>def ROS' + interface['name'] + "_" + method['name'] + '(self' + paramStrA + "):")
				else:
					cog.outl('<TABHERE>def ' + interface['name'] + "_" + method['name'] + '(self' + paramStrA + "):")
				if method['return'] != 'void': cog.outl("<TABHERE><TABHERE>ret = "+method['return']+'()')
				cog.outl("<TABHERE><TABHERE>#")
				cog.outl("<TABHERE><TABHERE>#subscribesToCODE")
				cog.outl("<TABHERE><TABHERE>#")
				if len(outValues) == 0:
					cog.outl("<TABHERE><TABHERE>pass\n")
				elif len(outValues) == 1:
					if method['return'] != 'void':
						cog.outl("<TABHERE><TABHERE>return ret\n")
					else:
						cog.outl("<TABHERE><TABHERE>"+outValues[0][1]+" = "+replaceTypeCPP2Python(outValues[0][0])+"()")
						cog.outl("<TABHERE><TABHERE>return "+outValues[0][1]+"\n")
				else:
					for v in outValues:
						if v[1] != 'ret':
							cog.outl("<TABHERE><TABHERE>"+v[1]+" = "+replaceTypeCPP2Python(v[0])+"()")
					first = True
					cog.out("<TABHERE><TABHERE>return [")
					for v in outValues:
						if not first: cog.out(', ')
						cog.out(v[1])
						if first:
							first = False
					cog.out("]\n\n")

if component['implements']:
	cog.outl("# =============== Methods for Component Implements ==================")
	cog.outl("# ===================================================================")
	for imp in sorted(component['implements']):
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communication_is_ice(imp):
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						cog.outl('<TABHERE>def ROS' + interface['name'] + "_" + method['name'] + "(self, req):")
						cog.outl("<TABHERE><TABHERE>#")
						cog.outl("<TABHERE><TABHERE># implementCODE")
						cog.outl("<TABHERE><TABHERE># Example ret = req.a + req.b")
						cog.outl("<TABHERE><TABHERE>#")
						cog.outl("<TABHERE><TABHERE>return "+method['name']+"Response(ret)")
		else:
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						outValues = []
						if method['return'] != 'void':
							outValues.append([method['return'], 'ret'])
						paramStrA = ''
						for p in method['params']:
							if p['decorator'] == 'out':
								outValues.append([p['type'], p['name']])
							else:
								paramStrA += ', ' +  p['name']
						cog.outl('')
						cog.outl('<TABHERE>#')
						cog.outl('<TABHERE># ' + method['name'])
						cog.outl('<TABHERE>#')
						cog.outl('<TABHERE>def ' + interface['name'] + "_" + method['name'] + '(self' + paramStrA + "):")
						if method['return'] != 'void': cog.outl("<TABHERE><TABHERE>ret = "+method['return']+'()')
						cog.outl("<TABHERE><TABHERE>#")
						cog.outl("<TABHERE><TABHERE># implementCODE")
						cog.outl("<TABHERE><TABHERE>#")
						if len(outValues) == 0:
							cog.outl("<TABHERE><TABHERE>pass\n")
						elif len(outValues) == 1:
							if method['return'] != 'void':
								cog.outl("<TABHERE><TABHERE>return ret\n")
							else:
								cog.outl("<TABHERE><TABHERE>"+outValues[0][1]+" = "+replaceTypeCPP2Python(outValues[0][0])+"()")
								cog.outl("<TABHERE><TABHERE>return "+outValues[0][1]+"\n")
						else:
							for v in outValues:
								if v[1] != 'ret':
									cog.outl("<TABHERE><TABHERE>"+v[1]+" = "+replaceTypeCPP2Python(v[0])+"()")
							first = True
							cog.out("<TABHERE><TABHERE>return [")
							for v in outValues:
								if not first: cog.out(', ')
								cog.out(v[1])
								if first:
									first = False
							cog.out("]\n\n")
	cog.outl("# ===================================================================")
	cog.outl("# ===================================================================\n")
]]]
[[[end]]]
