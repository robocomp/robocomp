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

ice_requires,ice_impliments = False,False
for require in component['requires']:
	req = require
	while type(req) != type(''):
		req = req[0]
	if communicationIsIce(req):
		ice_requires=True
		break
for impa in component['implements']:
	if type(impa) == str:
		imp = impa
	else:
		imp = impa[0]
	if communicationIsIce(imp):
		ice_impliments =True
		break
use_rcmaster = (ice_requires or ice_impliments)


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

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

preStr = "-I"+ROBOCOMP+"/interfaces/ -I/opt/robocomp/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

[[[cog
for imp in component['recursiveImports']:
	module = IDSLParsing.gimmeIDSL(imp.split('/')[-1])
	incl = imp.split('/')[-1].split('.')[0]
	cog.outl('Ice.loadSlice(preStr+"'+incl+'.ice")')
	cog.outl('from '+module['name']+' import *')
if use_rcmaster:
	cog.outl('Ice.loadSlice(preStr+"RCMaster.ice")')
	cog.outl('from RoboCompRCMaster import *')

]]]
[[[end]]]


[[[cog
	for imp in component['implements']+component['subscribesTo']:
		if type(imp) == str:
			im = imp
		else:
			im = imp[0]
		if communicationIsIce(imp):
			cog.outl('from ' + im.lower() + 'I import *')
]]]
[[[end]]]

[[[cog
if component['usingROS'] == True:
	cog.outl('import rospy')
	cog.outl('from std_msgs.msg import *')
	for include in modulesList:
		cog.outl("from "+include['name']+" import "+include['strName'])
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
						srvIncludes[module['name']] = 'from '+module['name']+' import *'
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
						srvIncludes[module['name']] = 'from '+module['name']+' import *'
	for srv in srvIncludes.values():
		cog.outl(srv)
A()
if component['gui'] != 'none':
	cog.outl('try:')
	cog.outl('<TABHERE>from ui_mainUI import *')
	cog.outl('except:')
	cog.outl('<TABHERE>print "Can\'t import UI file. Did you run \'make\'?"')
	cog.outl('<TABHERE>sys.exit(-1)')
Z()
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

		self.name = mprx["name"]

[[[cog
if ice_requires:
	cog.outl('<TABHERE><TABHERE>self.proxyData = mprx["proxyData"]')

for req, num in getNameNumber(component['requires']):
	if type(req) == str:
		rq = req
	else:
		rq = req[0]
	if not communicationIsIce(req):
		cog.outl("<TABHERE><TABHERE>self."+rq.lower()+"_proxy = ServiceClient"+rq+"()")
	else:
		cog.outl("<TABHERE><TABHERE>self."+rq.lower()+"_proxy = self.proxyData['"+rq.lower()+num+"']['proxy']")

for pb, num in getNameNumber(component['publishes']):
	if type(pb) == str:
		pub = pb
	else:
		pub = pb[0]
	if communicationIsIce(pb):
		cog.outl("<TABHERE><TABHERE>self."+pub.lower()+num+"_proxy = mprx[\""+pub+"Pub"+num+"\"]")
	else:
		cog.outl("<TABHERE><TABHERE>self."+pub.lower()+"_proxy = Publisher"+pub+"()")
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
