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

from parseCDSL import *
includeDirectories = theIDSLPaths.split('#')
component = CDSLParsing.fromFile(theCDSL, includeDirectories=includeDirectories)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
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

import sys, os, traceback, time

from PySide import *
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

[[[cog

all_interfaces = component.get('subscribesTo', []) + component['implements']
for interface in all_interfaces:
	for method in pool.get_methods(interface):
		if communicationIsIce(interface):
			cog.outl('<TABHERE><TABHERE>self.' + method["name"] + 'Buffer = CallQueue()')

]]]
[[[end]]]


	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#computeCODE
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]

		[[[cog

def implCodeCompute(method, outValues, params):
	cog.outl('')
	cog.outl('<TABHERE><TABHERE># ' + method)
	cog.outl('<TABHERE><TABHERE>params, cid = self.'+method+'Buffer.pop()')
	cog.outl('<TABHERE><TABHERE>if cid is not None:')
	for param in params:
		cog.outl('<TABHERE><TABHERE><TABHERE>'+param[1]+' = params["'+param[1]+'"]')

	cog.outl("<TABHERE><TABHERE><TABHERE>#")
	cog.outl("<TABHERE><TABHERE><TABHERE>#Logic for " + method)
	cog.outl("<TABHERE><TABHERE><TABHERE>#")
	
	if len(outValues) == 0:
		cog.outl("<TABHERE><TABHERE><TABHERE>self."+method+"Buffer.set_finished(cid)\n")
	elif len(outValues) == 1:
		if method['return'] != 'void':
			cog.outl("<TABHERE><TABHERE><TABHERE>self."+method+"Buffer.set_finished(cid, ret)\n")
		else:
			cog.outl("<TABHERE><TABHERE><TABHERE>"+outValues[0][1]+" = "+replaceTypeCPP2Python(outValues[0][0])+"()")
			cog.outl("<TABHERE><TABHERE><TABHERE>return "+outValues[0][1]+"\n")
			cog.outl("<TABHERE><TABHERE><TABHERE>self."+method+"Buffer.set_finished(cid, "+outValues[0][1]+" )\n")
	else:
		for v in outValues:
			if v[1] != 'ret':
				cog.outl("<TABHERE><TABHERE><TABHERE>"+v[1]+" = "+replaceTypeCPP2Python(v[0])+"()")
		first = True
		cog.out("<TABHERE><TABHERE><TABHERE>self."+method+"Buffer.set_finished(cid, [")
		for v in outValues:
			if not first: cog.out(', ')
			cog.out(v[1])
			if first:
				first = False
		cog.out("])\n")

all_interfaces = component.get('subscribesTo', []) + component['implements']
for interface in all_interfaces:
	for method in pool.get_methods(interface):
		if communicationIsIce(interface):
			implCodeCompute(method['name'], method['outValues'], method['params'])

]]]
[[[end]]]

		return True
