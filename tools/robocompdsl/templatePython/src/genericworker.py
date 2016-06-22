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
from parseSMDSL import *
component = CDSLParsing.fromFile(theCDSL)
sm = SMDSLparsing.fromFile(component['statemachine'])
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
[[[cog
#if sm is not "none":
	#cog.outl("<TABHERE>QtCore.__metaclass__  =  ABCMeta")
]]]
[[[end]]]

	kill = QtCore.Signal()
[[[cog
if component['statemachine'] != 'none':
    codsignals = ""
    if sm['machine']['contents']['transition'] != "none":
        for transi in sm['machine']['contents']['transition']:
            for dest in transi['dest']:
                codsignals += "<TABHERE>" + transi['src'] + "to" + dest + " = QtCore.Signal()\n"
    if sm['substates']!="none":
        for substates in sm['substates']:
            if substates['contents']['transition'] != "none":
                for transi in substates['contents']['transition']:
                    for dest in transi['dest']:
                        codsignals += "<TABHERE>" + transi['src'] + "to" + dest + " = QtCore.Signal()\n"
    cog.outl("#Signals for State Machine")
    cog.outl(codsignals)
    cog.outl("#-------------------------")
]]]
[[[end]]]

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
[[[cog
if sm is not "none":
	codStateMachine = ""
	codQState = ""
	codQStateParallel = ""
	codQFinalState = ""

	Machine = sm['machine']['name']
	codStateMachine = "<TABHERE><TABHERE>self." + Machine + "= QtCore.QStateMachine()"

	if sm['machine']['contents']['states'] is not "none":
		for state in sm['machine']['contents']['states']:
			aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(self." + Machine + ")\n"
			if sm['substates'] is not "none":
				for substates in sm['substates']:
					if state == substates['parent']:
						if substates['parallel'] is "parallel":
							aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(QtCore.QState.ParallelStates, self." + Machine +")\n"
							break
			if "ParallelStates" in aux:
				codQStateParallel += aux
			else:
				codQState += aux
	if sm['machine']['contents']['initialstate'] != "none":
		state = sm['machine']['contents']['initialstate'][0]
		aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(self." + Machine +")\n"
		if sm['substates'] is not "none":
			for substates in sm['substates']:
				if state == substates['parent']:
					if substates['parallel'] is "parallel":
						aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(QtCore.QState.ParallelStates,self." + Machine +")\n"
						break
		if "ParallelStates" in aux:
			codQStateParallel += aux
		else:
			codQState += aux
	if sm['machine']['contents']['finalstate'] != "none":
		state = sm['machine']['contents']['finalstate'][0]
		codQFinalState += "<TABHERE><TABHERE>self." + state + " = QtCore.QFinalState(self." + Machine +")\n"
	cog.outl("#State Machine")
	cog.outl(codStateMachine)
	cog.outl(codQState)
	cog.outl(codQFinalState)
	cog.outl(codQStateParallel)
	codStateMachine = ""
	codQState = ""
	codQStateParallel = ""
	codQFinalState = ""
	if sm['substates'] != "none":
		for substates in sm['substates']:
			if substates['contents']['states'] is not "none":
				for state in substates['contents']['states']:
					aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(self." + substates['parent'] + ")\n"
					for sub in sm['substates']:
						if state == sub['parent']:
							if sub['parallel'] is "parallel":
								aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(QtCore.QState.ParallelStates, self." + substates['parent'] + ")\n"
								break
					if "ParallelStates" in aux:
						codQStateParallel += aux
					else:
						codQState += aux
			if substates['contents']['initialstate'] != "none":
				aux = "<TABHERE><TABHERE>self." + substates['contents']['initialstate'] + " = QtCore.QState(self." + substates['parent'] + ")\n"
				for sub in sm['substates']:
					if state == sub['parent']:
						if sub['parallel'] is "parallel":
							aux = "<TABHERE><TABHERE>self." + state + " = QtCore.QState(QtCore.QState.ParallelStates, self." + substates['parent'] + ")\n"
							break
				if "ParallelStates" in aux:
					codQStateParallel += aux
				else:
					codQState += aux
			if substates['contents']['finalstate'] != "none":
				codQFinalState += "<TABHERE><TABHERE>self." + substates['contents']['finalstate'] + " = QtCore.QFinalState(self." + substates['parent'] + ")\n"
			cog.outl(codStateMachine)
			cog.outl(codQState)
			cog.outl(codQFinalState)
			cog.outl(codQStateParallel)
			codStateMachine = ""
			codQState = ""
			codQStateParallel = ""
			codQFinalState = ""
	cog.outl("#------------------")

	codaddTransition = ""
	codaddState = ""
	codConnect = ""
	codsetInitialState = ""
	if sm['machine']['contents']['transition'] != "none":
		for transi in sm['machine']['contents']['transition']:
			for dest in transi['dest']:
				codaddTransition += "<TABHERE><TABHERE>self." + transi['src'] + ".addTransition(self." + transi['src'] + "to" + dest+", self." + dest + ")\n"
	if sm['substates'] != "none":
		for substates in sm['substates']:
			if substates['contents']['transition'] != "none":
				for transi in substates['contents']['transition']:
					for dest in transi['dest']:
						codaddTransition += "<TABHERE><TABHERE>self." + transi['src'] + ".addTransition(self." + transi['src'] + "to" + dest+", self." + dest + ")\n"
	for state in sm['machine']['contents']['states']:
		codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
	if sm['machine']['contents']['initialstate'][0] is not "none":
		state = sm['machine']['contents']['initialstate'][0]
		codsetInitialState += "<TABHERE><TABHERE>self." + sm['machine']['name'] +  ".setInitialState(self." + state +")\n"
		codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
	if sm['machine']['contents']['finalstate'][0] is not "none":
		state = sm['machine']['contents']['finalstate'][0]
		codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
	if sm['substates'] != "none":
		for substates in sm['substates']:
			if substates['contents']['initialstate'] is not "none":
				state = substates['contents']['initialstate']
				codsetInitialState += "<TABHERE><TABHERE>self." + substates['parent'] +  ".setInitialState(self." + state +")\n"
				codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
			if substates['contents']['finalstate'] is not "none":
				state = substates['contents']['finalstate']
				codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
			if substates['contents']['states'] is not "none":
				for state in substates['contents']['states']:
					codConnect += "<TABHERE><TABHERE>self." + state + ".entered.connect(self.fun_" + state + ")\n"
	cog.outl("#Initialization State machine")
	cog.outl(codaddTransition)
	cog.outl(codaddState)
	cog.outl(codConnect)
	cog.outl(codsetInitialState)
	cog.outl("#------------------")
]]]
[[[end]]]
		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

[[[cog
if component['statemachine'] != 'none':
	codVirtuals = ""
	codcompsubclas = ""
	for state in sm['machine']['contents']['states']:
		codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + state + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + state + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
	if sm['machine']['contents']['initialstate'] != "none":
		codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + \
					   sm['machine']['contents']['initialstate'][0] + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + sm['machine']['contents']['initialstate'][0] + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
	if sm['machine']['contents']['finalstate'] != "none":
		codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + \
					   sm['machine']['contents']['finalstate'][0] + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + sm['machine']['contents']['finalstate'][0] + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
	if sm['substates'] != "none":
		for substates in sm['substates']:
			if substates['contents']['states'] is not "none":
				for state in substates['contents']['states']:
					codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + state + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + state + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
			if substates['contents']['initialstate'] != "none":
				codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + \
							   substates['contents']['initialstate'] + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + substates['contents']['initialstate'] + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
			if substates['contents']['finalstate'] != "none":
				codVirtuals += "<TABHERE>@QtCore.Slot()\n<TABHERE>def fun_" + \
							   substates['contents']['finalstate'] + '(self):\n<TABHERE><TABHERE>print "Error: lack fun_' + substates['contents']['finalstate'] + ' in Specificworker"\n<TABHERE><TABHERE>sys.exit(-1)\n\n'
		cog.outl("#Slots funtion State Machine")
		cog.outl(codVirtuals)
		cog.outl("#-------------------------")

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
