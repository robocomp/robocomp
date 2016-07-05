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
component = CDSLParsing.fromFile(theCDSL)
if component == None:
	print('Can\'t locate', theCDSLs)
	sys.exit(1)

from parseIDSL import *
pool = IDSLPool(theIDSLs)
modulesList = pool.rosModulesImports()

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

import sys
from PySide import *

[[[cog
if component['usingROS'] == True:
	cog.outl('import rospy')
	cog.outl('from std_msgs.msg import *')
	for include in modulesList:
		cog.outl("from "+include['name']+".msg import "+include['strName'])
	srvIncludes = {}
	for imp in component['requires']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communicationIsIce(imp):
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						srvIncludes[module['name']] = 'from '+module['name']+'.srv import *'
	for imp in component['implements']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if not communicationIsIce(imp):
			module = pool.moduleProviding(im)
			for interface in module['interfaces']:
				if interface['name'] == im:
					for mname in interface['methods']:
						srvIncludes[module['name']] = 'from '+module['name']+'.srv import *'
	for srv in srvIncludes.values():
		cog.outl(srv)
A()
if component['gui'] != 'none':
	cog.outl('')
	cog.outl('try:')
	cog.outl('<TABHERE>from ui_mainUI import *')
	cog.outl('except:')
	cog.outl('<TABHERE>print "Can\'t import UI file. Did you run \'make\'?"')
	cog.outl('<TABHERE>sys.exit(-1)')
Z()
]]]
[[[end]]]

[[[cog
if component['usingROS'] == True:
	#CREANDO CLASES PARA LOS PUBLISHERS
	for imp in component['publishes']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			cog.outl("#class for rosPublisher")
			cog.outl("class Publisher"+nname+"():")
			cog.outl("<TABHERE>def __init__(self):")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						for p in method['params']:
							s = "\""+nname+"_"+mname+"\""
							if p['type'] in ('float','int','uint'):
								cog.outl("<TABHERE><TABHERE>self.pub_"+mname+" = rospy.Publisher("+s+", "+p['type'].capitalize()+"32, queue_size=1000)")
							elif p['type'] == 'string':
								cog.outl("<TABHERE><TABHERE>self.pub_"+mname+" = rospy.Publisher("+s+", String, queue_size=1000)")
							elif '::' in p['type']:
								cog.outl("<TABHERE><TABHERE>self.pub_"+mname+" = rospy.Publisher("+s+", "+p['type'].split('::')[1]+", queue_size=1000)")
							else:
								cog.outl("<TABHERE><TABHERE>self.pub_"+mname+" = rospy.Publisher("+s+", "+p['type']+", queue_size=1000)")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						for p in method['params']:
							cog.outl("<TABHERE>def "+mname+"(self, "+p['name']+"):")
							cog.outl("<TABHERE><TABHERE>self.pub_"+mname+".publish("+p['name']+")")
	#CREANDO CLASES PARA LOS REQUIRES
	for imp in component['requires']:
		nname = imp
		while type(nname) != type(''):
			nname = nname[0]
		module = pool.moduleProviding(nname)
		if module == None:
			print ('\nCan\'t find module providing', nname, '\n')
			sys.exit(-1)
		if not communicationIsIce(imp):
			cog.outl("#class for rosServiceClient")
			cog.outl("class ServiceClient"+nname+"():")
			cog.outl("<TABHERE>def __init__(self):")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname] #for p in method['params']:
						s = "\""+nname+"_"+mname+"\""
						cog.outl("<TABHERE><TABHERE>self.srv_"+mname+" = rospy.ServiceProxy("+s+", "+mname+")")
			for interface in module['interfaces']:
				if interface['name'] == nname:
					for mname in interface['methods']:
						method = interface['methods'][mname]
						paramStrA = ''
						for p in method['params']:
							# delim
							if paramStrA == '': paramStrA = p['name']
						cog.outl("<TABHERE>def "+mname+"(self, "+paramStrA+"):")
						cog.outl("<TABHERE><TABHERE>return self.srv_"+mname+"("+paramStrA+")")
]]]
[[[end]]]

class GenericWorker(
[[[cog
A()
if component['gui'] != 'none':
	cog.out('QtGui.'+component['gui'][1])
else:
	cog.out('QtCore.QObject')
Z()
]]]
[[[end]]]
):
	kill = QtCore.Signal()


	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


[[[cog
for req in component['requires']:
	if type(req) == str:
		rq = req
	else:
		rq = req[0]
	if communicationIsIce(req):
		cog.outl("<TABHERE><TABHERE>self."+rq.lower()+"_proxy = mprx[\""+rq+"Proxy\"]")
	else:
		cog.outl("<TABHERE><TABHERE>self."+rq.lower()+" = ServiceClient"+nname+"()")

for pb in component['publishes']:
	if type(pb) == str:
		pub = pb
	else:
		pub = pb[0]
	if communicationIsIce(pb):
		cog.outl("<TABHERE><TABHERE>self."+pub.lower()+" = mprx[\""+pub+"Pub\"]")
	else:
		cog.outl("<TABHERE><TABHERE>self."+pub.lower()+" = Publisher"+nname+"()")
]]]
[[[end]]]

[[[cog
A()
if component['gui'] != 'none':
	cog.outl("<TABHERE><TABHERE>self.ui = Ui_guiDlg()")
	cog.outl("<TABHERE><TABHERE>self.ui.setupUi(self)")
	cog.outl("<TABHERE><TABHERE>self.show()")
Z()
]]]
[[[end]]]
		
		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)


	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print "Period changed", p
		Period = p
		timer.start(Period)
