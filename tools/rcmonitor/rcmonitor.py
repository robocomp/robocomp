#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
#  ------------------------
#  ----- rcompmonitor -----
#  ------------------------
#  An ICE component monitor.
#
#    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#
#
# \mainpage RoboComp::monitorComp
#
# \section intro_sec Introduction
#
# MonitorComp is a component to check if some random component is working fine. This application removes the need to create a new component just to check other component.
#
# \section interface_sec Interface
#
#
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# monitorComp does not have specific software dependencies.
#
# \subsection install2_ssec Compile and install
# cd Tools/monitorComp
# <br>
# It's an python component so no need compilation
#
# \section guide_sec User guide
#
#
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file is the most important issue to get replayComp working. First, we will set the mode in which replayComp will run by seting the variable 'Mode', is can be whether 'REPLAY' or 'CAPTURE'. In both cases the variable "File" will specify the file in which data will be written or should be read from.
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/monitorComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
# If running in replay mode you can (un)pause the execution or store frames by pressing the corresponding button. You can also (un)hide the application by clicking on the icon you will se in the system tray.
#
#



global_ic = None
force_host = ''
force_port = ''

BasePoseCodeTemplate = '''# Base Pose Template\nimport Ice\nimport sys\nclass C():\n  def __init__(self, endpoint, modules):\n    self.ic = Ice.initialize(sys.argv)\n    self.mods = modules\n    self.prx = self.ic.stringToProxy(endpoint)\n    self.proxy = self.mods['RoboCompDifferentialRobot'].DifferentialRobotPrx.checkedCast(self.prx)\n    self.x = 0\n    self.z = 0\n    self.a = 0\n  def job(self):\n    pose = self.proxy.getBasePose()\n    print pose\n    print len(pose)\n    return ['pose', [ pose[0]/10, pose[1]/10, pose[2] ] ]\n'''
CameraRGBCodeTemplate = '''# Camera RGB Image Template\nimport Ice\nimport sys\nclass C():\n  def __init__(self, endpoint, modules):\n    self.ic = Ice.initialize(sys.argv)\n    self.mods = modules\n    self.prx = self.ic.stringToProxy(endpoint)\n    self.proxy = self.mods['RoboCompCamera'].CameraPrx.checkedCast(self.prx)\n    self.params = self.proxy.getCamParams()\n  def job(self):\n    vector, hState, bState = self.proxy.getRGBPackedImage(5)\n    if len(vector) == 0:\n     print 'Error retrieving images!'\n    return ['rgbImage', [ vector, self.params.width, self.params.height*self.params.numCams ] ]\n'''
CameraGreyCodeTemplate = '''# Camera grey Image Template\nimport Ice\nimport sys\nclass C():\n  def __init__(self, endpoint, modules):\n    self.ic = Ice.initialize(sys.argv)\n    self.mods = modules\n    self.prx = self.ic.stringToProxy(endpoint)\n    self.proxy = self.mods['RoboCompCamera'].CameraPrx.checkedCast(self.prx)\n    self.params = self.proxy.getCamParams()\n  def job(self):\n    vector, hState, bState = self.proxy.getYImage(5)\n    if len(vector) == 0:\n     print 'Error retrieving images!'\n    return ['greyImage', [ vector, self.params.width, self.params.height*self.params.numCams ] ]\n'''
CustomCodeTemplate = '''# Custom Template\nimport Ice\nimport sys\n\nfrom PyQt4.QtCore import *\nfrom PyQt4.QtGui import *\nfrom PyQt4.Qt import *\n\nclass C(QWidget):\n  def __init__(self, endpoint, modules):\n    QWidget.__init__(self)\n    self.ic = Ice.initialize(sys.argv)\n    self.mods = modules\n    self.prx = self.ic.stringToProxy(endpoint)\n    self.proxy = self.mods['RoboCompCamera'].CameraPrx.checkedCast(self.prx)\n    self.measures = range(33)\n    self.params = self.proxy.getCamParams();\n    self.job()\n\n  def job(self):\n    # Remote procedure call\n    self.image, head, bstate = self.proxy.getRGBPackedImage(0) # vector, head, bState\n\n    # Store pos measure\n    self.measures.pop(0)\n    self.measures.append(head.tilt.pos)\n\n  def paintEvent(self, event=None):\n    painter = QPainter(self)\n    painter.setRenderHint(QPainter.Antialiasing, True)\n    # Draw image\n    qimage = QImage(self.image, self.params.width, self.params.height, QImage.Format_RGB888)\n    painter.drawImage(QPointF(0, 0), qimage)\n    # Draw signal\n    for idx in range(len(self.measures)-1):\n      painter.drawLine(idx*10, (self.height()/2)-(self.measures[idx]*100), (idx+1)*10, (self.height()/2)-(self.measures[idx+1]*100))\n\n    painter.end()\n\n'''
SignalCodeTemplate = '''# Signal Template\nimport Ice\nimport sys\nclass C():\n  def __init__(self, endpoint, modules):\n    self.ic = Ice.initialize(sys.argv)\n    self.mods = modules\n    self.prx = self.ic.stringToProxy(endpoint)\n    self.proxy = self.mods['RoboCompCamMotion'].CamMotionPrx.checkedCast(self.prx)\n  def job(self):\n    hState = self.proxy.getHeadState()\n    print hState.tilt.pos\n    return ['signal', [ hState.tilt.pos, 200 ] ]\n'''
#
# CODE BEGINS
#
import sys, time, traceback, os, math, random, threading, time, new, Ice

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.Qt import *
from ui_formMonitor import Ui_Form
from ui_openA import Ui_OpenA
from ui_period import Ui_Period

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


def exists(parent, name):
	try:
		eval('parent.'+name)
		return True
	except:
		return False


def importCode(code, name):
	module = new.module(name)
	exec code in module.__dict__
	return module


def getParamIdx():
	argIdx = range(len(sys.argv))
	for i in range(2):
		try:
			argIdx.remove(i)
		except:
			pass
	try:
		argIdx.pop()
	except:
		pass
	return argIdx


class SliceReader():
	def __init__(self, slicePath, sliceOpts=''):
		self.slicePath = slicePath
		self.sliceOpts = sliceOpts
		self.RoboComps = dict()

		loadCommand = self.sliceOpts+' '+self.slicePath
		print 'Loading SLICE file: "'+loadCommand+'"'
		print '1'
		modulesA = [ module for module in sys.modules ]
		print '2'
		try:
			print '3', loadCommand
			Ice.loadSlice(loadCommand)
			print '4'

			modulesB = [ module for module in sys.modules ]
			newModules = [ module for module in modulesB if not module in modulesA ]

			for newModule in newModules:
				print '  New module: ' + newModule
				self.RoboComps[str(newModule)] = __import__(newModule)
			try:
				global global_ic
				global_ic = Ice.initialize(sys.argv)
			except:
				print '  global_ic'
		except:
			traceback.print_exc(file=sys.stdout)
			print 'Error loading ' + loadCommand + '.'
		finally:
			print 'Done loading ' + self.slicePath + '.'


class PeriodConfig(QDialog):
	def __init__(self, parent=None):
		QDialog.__init__(self, parent)
		self.ui = Ui_Period()
		self.ui.setupUi(self)
		self.setModal(True)
		self.avoidRecursion = 1
		self.value = self.ui.msBox.value()
		self.connect(self.ui.msBox, SIGNAL('valueChanged(int)'), self.msChanged)
		self.connect(self.ui.hzBox, SIGNAL('valueChanged(double)'), self.hzChanged)
		self.connect(self.ui.buttonBox, SIGNAL('accepted()'), self.slotAccepted)
		self.connect(self.ui.buttonBox, SIGNAL('rejected()'), self.slotRejected)

	def slotAccepted(self):
		self.emit(SIGNAL("done()"))
		self.hide()
	def slotRejected(self):
		self.hide()
	def msChanged(self, value):
		if self.avoidRecursion == 1:
			self.avoidRecursion -= 1
			self.ui.hzBox.setValue(1000./value)
			self.avoidRecursion += 1
			self.value = value
	def hzChanged(self, value):
		if self.avoidRecursion == 1:
			self.avoidRecursion -= 1
			self.ui.msBox.setValue(int(1000./value))
			self.avoidRecursion += 1
			self.value = int(1000./value)


class OpenConnection(QDialog):
	def __init__(self, parent=None):
		QDialog.__init__(self, parent)
		self.achoParent = parent
		self.ui = Ui_OpenA()
		self.ui.setupUi(self)
		self.setModal(True)
		#self.ui.tab1.setLayout(self.ui.verticalLayout)
		#self.ui.tab2.setLayout(self.ui.gridLayout)
		#self.ui.sliceBox.setLayout(self.ui.gl1)
		#self.ui.endpointBox.setLayout(self.ui.hl2)
		self.ui.tabWidget.setTabEnabled(0, True)
		self.ui.tabWidget.setTabEnabled(1, False)
		self.slicePath = str(self.ui.slicePathWidget.text())
		self.endpoint = ''
		self.connect(self.ui.slicePathWidget, SIGNAL('textChanged(QString)'), self.sliceChanged)
		self.connect(self.ui.button, SIGNAL('clicked()'), self.slotSelect)
		self.connect(self.ui.buttonBox1, SIGNAL('accepted()'), self.slotAccepted1)
		self.connect(self.ui.buttonBox1, SIGNAL('rejected()'), self.slotRejected)
		self.ui.buttonBox1.setFocus()
	def loadConfig(self, config, root):
		# Automatically set slice path
		self.slicePath = ROBOCOMP + "/" + config[0]
		self.ui.slicePathWidget.setText(self.slicePath)
		# Automatically set slice options
		pathsplit = self.slicePath.rsplit('/', 1)
		if len(pathsplit)>1:
			self.ui.sliceOpts.append(self.slicePath.rsplit('/', 1)[0]+'/')
		# Automatically set endpoint name
		self.ui.endpointName.setText(config[1])
		# Automatically set host name
		self.ui.hostName.setText(config[2])
		# Automatically set endpoint protocol
		if (config[3].split()[0] == 'tcp'):
			self.ui.endpointProtocol.setCurrentIndex(0)
		elif (config[3].split()[0] == 'udp'):
			self.ui.endpointProtocol.setCurrentIndex(1)
		elif (config[3].split()[0] == 'ws'):
			self.ui.endpointProtocol.setCurrentIndex(2)
		else:
			print 'Wrong protocol:',config[3].split()[0]
			sys.exit()
		# Automatically set endpoint port number
		self.ui.endpointPort.setValue(int(config[4]))
		# Automatically set code
		codePath = root
		if len(codePath)>0:
			codePath += "/"
		codePath += config[5]
		try:
			print 'Opening code file <'+codePath+'>'
			f = open(codePath, 'r')
		except:
			print 'Cannot open code file <'+codePath+'>'
			sys.exit(1)
		self.slotAccepted1()
		self.ui.textEdit.setText('')
		#print '<CODE'
		for line in f.readlines():
			self.ui.textEdit.append(line.rstrip())
			#print line,
		#print 'CODE>'
		self.slotAccepted2()
	def sliceChanged(self, qstr):
		if self.ui.endpointName.text().size() == 0:
			filename = qstr.split('/')[-1]
			ifname = filename.split('.ice')[0].toLower()
			self.ui.endpointName.setText(ifname)
	def resizeEvent(self, ev):
		newSize = ev.size()
		newSize.setWidth(newSize.width() - 10)
		newSize.setHeight(newSize.height() - 10)
		self.ui.tabWidget.resize(newSize)
		self.ui.tabWidget.move(5, 5)
	def slotSelect(self):
		self.slicePath = str(QFileDialog.getOpenFileName(self, 'Select an slice file (*.ice)', '.', '*.ice' ))
		self.ui.slicePathWidget.setText(self.slicePath)
		pathsplit = self.slicePath.rsplit('/', 1)
		if len(pathsplit)>1:
			self.ui.sliceOpts.append(+'/')
	def slotAccepted1(self):
		self.endpoint = str(self.ui.endpointName.text() + ': ')
		self.endpoint = self.endpoint + str(self.ui.endpointProtocol.currentText() + ' -p ' + self.ui.endpointPort.text())
		if len(str(self.ui.hostName.text())) != 0:
			self.endpoint = self.endpoint + str(' -h ' + self.ui.hostName.text() + ' ')
		#print self.endpoint
		sliceOpts = ''
		for directory in str(self.ui.sliceOpts.toPlainText()).split('\n'):
			if len(directory)>0:
				sliceOpts = sliceOpts + ' -I' + directory + ' '
		sliceOpts = sliceOpts + ' -I. '
		sliceOpts = sliceOpts + ' -I' + ROBOCOMP + '/interfaces '
		sliceOpts = sliceOpts + ' --all'
		print 'Slice options:', sliceOpts
		self.sr = SliceReader(self.slicePath, sliceOpts)

		self.importedSymbols = ''
		proxyName = ''
		for key in self.sr.RoboComps.keys():
			for element in dir(self.sr.RoboComps[key]):
				if element[:3] != '_t_' and element[:2] != '__':
					self.importedSymbols += element+'\n'
				if element[-3:] == 'Prx' and element[:3] == '_t_':
					proxyName = element[3:]
			self.ui.symbolsText.insertPlainText(self.importedSymbols)
			self.importedMethods = ''
			if proxyName != '':
				proxyClassName = 'self.sr.RoboComps[\''+key+'\'].' + proxyName
				proxyUncasted = eval(proxyClassName)
				for element in dir(proxyUncasted):
					if element[:4] != 'ice_' and element[:2] != '__':
						self.importedMethods += element+'\n'
				self.ui.methodsText.insertPlainText(self.importedMethods)

		self.ui.tabWidget.setTabEnabled(0, False)
		self.ui.tabWidget.setTabEnabled(1, True)
		self.connect(self.ui.buttonBox2, SIGNAL('accepted()'), self.slotAccepted2)
		self.connect(self.ui.buttonBox2, SIGNAL('rejected()'), self.slotRejected)
		self.ui.tabWidget.setCurrentIndex(1)
		self.achoParent.ui.menuTemplates.setEnabled(True)
		self.ui.buttonBox2.setFocus()
	def slotAccepted2(self):
		self.achoParent.ui.menuTemplates.setEnabled(False)
		src = str(self.ui.textEdit.toPlainText())
		self.module = importCode(src, 'modulillo')
		self.hide()
		self.emit(SIGNAL("done()"))
	def slotRejected(self):
		self.hide()


class DrawData(QWidget):
	def __init__(self):
		QWidget.__init__(self)
		self.t = ''
		self.data = None
	def setData(self, t, data):
		self.t = t
		w = float(self.width())
		if t=='signal':
			if self.data == None: self.data = [ data[1] ]
			else: self.data[0] = data[1]
			if len(self.data)-1>w/4:
				self.data.pop(1)
			self.data.append(float(data[0]))
		else:
			self.data = data
 	def paintEvent(self, event=None):
		painter = QPainter(self)
		painter.setRenderHint(QPainter.Antialiasing, True)
		w = float(self.width())
		h = float(self.height())

		if self.t == 'pose':
			# Draw head
			incX = (1+math.cos(self.data[2]))*10.-5
			incY = (1+math.sin(self.data[2]))*10.-5
			painter.setBrush(QColor(0, 0, 255))
			painter.setPen(QColor(0, 0, 255))
			painter.drawEllipse(self.data[1]+w/2, self.data[0]+h/2, 20, 20)
			# Draw body
			painter.setBrush(QColor(255, 0, 0))
			painter.setPen(QColor(255, 0, 0))
			painter.drawEllipse((self.data[1]+incX)+w/2, (self.data[0]+incY)+h/2, 10, 10)
		if self.t == 'rgbImage':
			image = QImage(self.data[0], self.data[1], self.data[2], QImage.Format_RGB888)
			painter.drawImage(QPointF(0, 0), image)
			painter.drawLine( QLine(0, int(self.data[2]/2), self.data[1], int(self.data[2]/2)) )
			if self.data[2] > self.data[1]:
				painter.drawLine( QLine(0, int(self.data[2]/(4./1.)), self.data[1], int(self.data[2]/(4./1.))) )
				painter.drawLine( QLine(0, int(self.data[2]/(4./3.)), self.data[1], int(self.data[2]/(4./3.))) )
			painter.drawLine( QLine(int(self.data[1]/2), 0, int(self.data[1]/2), self.data[2]) )
		if self.t == 'greyImage':
			image = QImage(self.data[0], self.data[1], self.data[2], QImage.Format_Indexed8)
			for i in range(256):
				image.setColor(i, QColor(i,i,i).rgb())
			painter.drawImage(QPointF(0, 0), image)
		if self.t == 'signal':
			l = len(self.data)-1
			painter.setPen(QColor(0, 0, 0))
			painter.drawLine(0, h/2, w, h/2)
			painter.setPen(QColor(0, 0, 255))
			for idx in range(l):
				pos = l-idx-1
				wpos = w-4.*idx
				painter.drawLine(wpos, (h/2.)-(self.data[pos+1]*self.data[0]), wpos-4., (h/2.)-(self.data[pos]*self.data[0]))
		if self.t == 'laser':
			pass

		painter.end


class RCOMPMonitor(QMainWindow):
	def __init__(self):
		QMainWindow.__init__(self)
		self.ui = Ui_Form()
		self.ui.setupUi(self)
#		self.setCentralWidget(self.ui.mdiArea)
		self.ui.menuTemplates.setEnabled(False)
		self.periodConfig = PeriodConfig(self)
		self.period = self.periodConfig.value
		self.shortcuts = list()
		sc = QShortcut(QKeySequence("Ctrl+W"), self)
		self.shortcuts.append(sc)
		self.connect(sc, SIGNAL("activated()"), self.triggerExit)
		sc = QShortcut(QKeySequence("Ctrl+O"), self)
		self.shortcuts.append(sc)
		self.connect(sc, SIGNAL("activated()"), self.triggerOpen)

		self.connect(self.ui.actionOpen, SIGNAL('triggered()'), self.triggerOpen)
		self.connect(self.ui.actionClose, SIGNAL('triggered()'), self.triggerClose)
		self.connect(self.ui.actionExit, SIGNAL('triggered()'), self.triggerExit)
		self.connect(self.ui.actionPeriod, SIGNAL('triggered()'), self.triggerPeriod)
		self.connect(self.ui.actionBase_pose, SIGNAL('triggered()'), self.triggerTemplateBase)
		self.connect(self.ui.actionCamera_RGB_Image, SIGNAL('triggered()'), self.triggerTemplateRGBImage)
		self.connect(self.ui.actionCamera_Grey_Image, SIGNAL('triggered()'), self.triggerTemplateGreyImage)
		self.connect(self.ui.actionCustom, SIGNAL('triggered()'), self.triggerTemplateCustom)
		self.connect(self.ui.actionSignal, SIGNAL('triggered()'), self.triggerTemplateSignal)
		self.connect(self.periodConfig, SIGNAL('done()'), self.periodChange)

		self.statusBar().showMessage('Click on \'Connection->Open\' to establish a new connection.')
		self.ticks = 0

		if len(sys.argv) > 1:
			global force_host
			global force_port
			try:
				print 'Opening configuration file', sys.argv[1]
				f = open(sys.argv[1], 'r')
			except:
				print 'Cannot open configuration file', sys.argv[1]
				sys.exit()
			argIdx = getParamIdx()
			print argIdx
			for idx in argIdx:
				if sys.argv[idx] == '-h':
					force_host = sys.argv[idx+1]
				elif sys.argv[idx] == '-p':
					force_port = sys.argv[idx+1]
			self.triggerOpen()
			openConfig = [ line.strip() for line in f.readlines() if len(line.strip()) > 0 and line.strip()[0] != '#' and len(line.strip()) > 0]
			self.periodConfig.msChanged(int(openConfig.pop(5)))
			if force_host != '': openConfig[2] = force_host
			if force_port != '': openConfig[4] = force_port
			root = os.path.dirname( sys.argv[1] )
			print ("FILE: " + sys.argv[1])
			print ("ROOT: " + root)
			self.openA.loadConfig(openConfig, root)
	def triggerOpen(self):
		self.openA = OpenConnection(self)
		self.subwindow = self.ui.mdiArea.addSubWindow(self.openA)
		self.subwindow.showMaximized()
		self.connect(self.openA, SIGNAL('done()'), self.doJob)
		self.openA.show()
		self.ui.menuTemplates.setEnabled(False)
	def triggerClose(self):
		if exists(self, 'openA'): self.openA.close()
		self.ui.menuTemplates.setEnabled(False)
	def triggerExit(self):
		self.close()
	def triggerPeriod(self):
			self.periodConfig.show()
	def triggerTemplateBase(self):
		if exists(self, 'openA'): self.openA.ui.textEdit.setText(BasePoseCodeTemplate)
	def triggerTemplateRGBImage(self):
		if exists(self, 'openA'): self.openA.ui.textEdit.setText(CameraRGBCodeTemplate)
	def triggerTemplateGreyImage(self):
		if exists(self, 'openA'): self.openA.ui.textEdit.setText(CameraGreyCodeTemplate)
	def triggerTemplateCustom(self):
		if exists(self, 'openA'): self.openA.ui.textEdit.setText(CustomCodeTemplate)
	def triggerTemplateSignal(self):
		if exists(self, 'openA'): self.openA.ui.textEdit.setText(SignalCodeTemplate)
	def startNewConnection(self):
		self.triggerClose()
	def doJob(self):
		self.subwindow.close()
		if exists(self, 'openA') and exists(self.openA.module, 'C'):
			self.doer = self.openA.module.C(self.openA.endpoint, self.openA.sr.RoboComps)
			self.openA.close()
			if issubclass(self.openA.module.C, QWidget):
				self.subwindow = self.ui.mdiArea.addSubWindow(self.doer)
				self.doer.setParent(self.subwindow)
				self.subwindow.showMaximized()
			else:
				self.drawer = DrawData()
				self.subwindow = self.ui.mdiArea.addSubWindow(self.drawer)
				self.subwindow.showMaximized()

			self.mySlot()
			self.timer = QTimer()
			self.timer.start(self.period)
			self.connect(self.timer, SIGNAL('timeout()'), self.mySlot)
	def mySlot(self):
		if exists(self.doer, 'job'):
			if exists(self, "drawer"):
				output = self.doer.job()
				if (output != None):
					self.drawer.setData(output[0], output[1])
					self.drawer.update()
			else:
				self.doer.job()
				self.doer.update()
			self.statusBar().showMessage(QString('Ticks: ')+QString.number(self.ticks))
			self.ticks = self.ticks + 1
	def periodChange(self):
		self.period = self.periodConfig.value
		if exists(self, 'timer'): self.timer.start(self.period)

if __name__ == '__main__':
	app = QApplication(sys.argv)
	clase = RCOMPMonitor()
	clase.show()
	app.exec_()

