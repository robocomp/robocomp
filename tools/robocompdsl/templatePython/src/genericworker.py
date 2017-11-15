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


[[[cog
for namea in component['requires']:
	if type(namea) == str:
		name = namea
	else:
		name = namea[0]
		cog.outl("<TABHERE><TABHERE>self."+name.lower()+"_proxy = mprx[\""+name+"Proxy\"]")
]]]
[[[end]]]

[[[cog
for pba in component['publishes']:
	if type(pba) == type(''):
		pb = pba
	else:
		pb = pba[0]
	cog.outl("<TABHERE><TABHERE>self."+pb.lower()+" = mprx[\""+pb+"Pub\"]")
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

[[[cog
	if len(component['publishes']) > 0 or len(component['subscribesTo']) > 0:
		cog.outl("""			
		<TABHERE><TABHERE>self._storm_timer = QtCore.QTimer(self)
		<TABHERE><TABHERE>self._storm_timer.timeout.connect(self.stormCheck)
		<TABHERE><TABHERE>self._storm_period = 100
		<TABHERE><TABHERE>self._storm_timer.start(self._storm_period)
		""")
]]]
[[[end]]]


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

[[[cog
	cog.outl("""
	<TABHERE>@QtCore.Slot()
	<TABHERE>def stormCheck(self):
	<TABHERE><TABHERE>try:
	<TABHERE><TABHERE><TABHERE>IceStorm.TopicManagerPrx.checkedCast(self.mprx["topicManager"])
	<TABHERE><TABHERE>except KeyError:
	<TABHERE><TABHERE><TABHERE>print "please set Storm Endpoints "
	<TABHERE><TABHERE>except:
	<TABHERE><TABHERE><TABHERE>print "STORM not running"
	""")
]]]
[[[end]]]