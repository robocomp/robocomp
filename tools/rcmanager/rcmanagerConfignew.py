#!/usr/bin/env python2.7
#
#  -----------------------
#  ----- rcmanager -----
#  -----------------------
#  An ICE component manager.
#

#    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
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


# Importamos el modulo libxml2
import libxml2, sys,threading
from PyQt4 import QtCore, QtGui, Qt
filePath = 'rcmanager.xml'


try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)


class VisualNode(QtGui.QGraphicsItem):##Visual Node GraphicsItem
	"""docstring for ClassName"""
	def __init__(self, view=None,Alias=None,parent=None):
		QtGui.QGraphicsItem.__init__(self)
		self.parent=parent
		self.view=view
		self.Alias=Alias
		self.pos=None
		self.IpColor=None
		self.aliveStatus=False
		self.setFlag(QtGui.QGraphicsItem.ItemIsMovable)
	def setIpColor(self,color=QtGui.QColor.fromRgb(0,255,0)):#To set the Ipcolor
		self.IpColor=color
	def setview(view):
		self.view=view
	def boundingRect(self):
		self.penWidth =2
		return QtCore.QRectF(-49,-49,98,98)
	def paint(self,painter,option=None,widget=None):
		self.paintMainShape(painter)
		self.drawStatus(painter)
		self.drawIcon(painter)
		self.writeAlias(painter)
	def setIcon(self,icon):
		self.Icon=icon
	def paintMainShape(self,painter): ##This will draw the basic shape of a node.The big square and its containing elements
		pen=QtGui.QPen(QtGui.QColor.fromRgb(0,0,0))
		pen.setWidth(3)
		painter.setPen(pen)
		brush=QtGui.QBrush(QtGui.QColor.fromRgb(94,94,94))
		painter.setBrush(brush)
		painter.drawRoundedRect(-49,-49,98,98,5,5)
		brush.setColor(QtGui.QColor.fromRgb(255,255,255))
		painter.setBrush(brush)
		self.TextRect=QtCore.QRect(-45,-45,90,20)##The rectangle shape on which the alias name will be dispalyed
		self.statusRect=QtCore.QRect(22,10,20,20)##The rectange shape on which the status of the node will be displayed
		self.IconRect=QtCore.QRect(-45,-20,60,64)## The rectange shape on which the Icon will be displayed
		#self.drawConnection(painter,"UP","INCOMMING")
		#self.drawConnection(painter,"RIGHT","OUTGOING")
		painter.drawRect(self.TextRect) ##Drawing the Alias display rectangle
		painter.setBrush(self.IpColor)  ## Drawing the Icon display rectangle
		painter.drawRect(self.IconRect)
		
		
	def drawConnection(self,painter,edge="UP",DIRECTION="INCOMMING",dist=0):##This is used to draw the exit and entry points of connections on the node
		if edge=="UP":
			if DIRECTION=="INCOMMING":
				Arrow=QtGui.QPixmap("share/rcmanager/downconnection.png")
				painter.drawPixmap(-49+21*(dist),-69,20,20,Arrow,0,0,0,0)
			elif DIRECTION=="OUTGOING":
				Arrow=QtGui.QPixmap("share/rcmanager/upconnection.png")
				painter.drawPixmap(-49+21*(dist),-69,20,20,Arrow,0,0,0,0)
		elif edge=="DOWN":
			if DIRECTION=="INCOMMING":
				Arrow=QtGui.QPixmap("share/rcmanager/upconnection.png")
				painter.drawPixmap(-49+21*(dist),49,20,20,Arrow,0,0,0,0)
			elif DIRECTION=="OUTGOING":
				Arrow=QtGui.QPixmap("share/rcmanager/downconnection.png	")
				painter.drawPixmap(-49+21*(dist),49,20,20,Arrow,0,0,0,0)
			
		elif edge=="LEFT":
			if DIRECTION=="INCOMMING":
				Arrow=QtGui.QPixmap("share/rcmanager/rightconnection.png")
				painter.drawPixmap(-69,-49+21*(dist),20,20,Arrow,0,0,0,0)
			elif DIRECTION=="OUTGOING":
				Arrow=QtGui.QPixmap("share/rcmanager/leftconnection.png")
				painter.drawPixmap(-69,-49+21*(dist),20,20,Arrow,0,0,0,0)
			
		elif edge=="RIGHT":
			if DIRECTION=="INCOMMING":
				Arrow=QtGui.QPixmap("share/rcmanager/leftconnection.png")
				painter.drawPixmap(49,-49+21*(dist),20,20,Arrow,0,0,0,0)
			elif DIRECTION=="OUTGOING":
				Arrow=QtGui.QPixmap("share/rcmanager/rightconnection.png")
				painter.drawPixmap(49,-49+21*(dist),20,20,Arrow,0,0,0,0)
	def drawIcon(self,painter):#Draw the Icon representing the purpose and draw its 
			painter.setBrush(self.IpColor)  ## Drawing the Icon display rectangle
			painter.drawRect(self.IconRect)
			painter.drawPixmap(-45,-20,60,60,self.Icon,0,0,0,0)
	def writeAlias(self,painter):##Draw the alias
			painter.drawText(self.TextRect,1," "+self.Alias) 	##Drawing the Alias name
	def drawStatus(self,painter):
		if self.parent.status:
			brush=QtGui.QBrush(QtGui.QColor.fromRgb(0,255,0)) ##Drawing the Status display rectangle
			painter.setBrush(brush)
			painter.drawRect(self.statusRect)
		else:
			brush=QtGui.QBrush(QtGui.QColor.fromRgb(255,0,0)) ##Drawing the Status display rectangle
			painter.setBrush(brush)
			painter.drawRect(self.statusRect)

class ComponentChecker(threading.Thread):#This will check the status of components
	def __init__(self,component):
		threading.Thread.__init__(self)
		self.component=component
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.daemon = True
		self.reset()
		self.exit = False
		self.alive = False
		self.aPrx = None
		try:
			ic=Ice.initialize()
			self.aPrx = ic.stringToProxy(self.componentendpoint)
			self.aPrx.ice_timeout(10)
		except:
			print "Error creating proxy to " + endpoint
			if len(endpoint) == 0:
				print 'please, provide an endpoint'
			raise
			
	def run(self):
		global global_ic
		while self.exit == False:
			try:
				self.aPrx.ice_ping()
				self.mutex.lock()
				if self.alive==False:
					self.changed()	
				self.alive = True
				self.mutex.unlock()
			except:
				self.mutex.lock()
				if self.alive==True:
					self.changed()
				self.alive = False
				self.mutex.unlock()
			time.sleep(0.5)
	def reset(self):
		self.mutex.lock()
		self.alive = False
		self.mutex.unlock()
	def isalive(self):
		self.mutex.lock()
		r = self.alive
		self.mutex.unlock()
		return r
	def stop(self):
		self.exit = True
	def runrun(self):
		if not self.isAlive(): self.start()
	def changed(self):
		self.component.status=not self.alive
		self.component.graphicsItem.update()
#		
# Component information container class.
#
class CompInfo:##This contain the general Information about the Components which is read from the files and created
	def __init__(self):
		self.endpoint = ''
		self.workingdir = ''
		self.compup = ''
		self.compdown = ''
		self.alias = ''
		self.dependences = []
		self.configFile = ''
		self.x = 0##This is not reliable >>Have to fix the bug
		self.y = 0##This is not reliable >>Have to fix the bug
		self.Ip=""
		self.IconFilePath=""
		self.status=False
		self.CheckItem=ComponentChecker(component=self)
		self.graphicsItem=VisualNode(parent=self)
	def __repr__(self):
		string = ''
		string = string + '[' + self.alias + ']:\n'
		string = string + 'endpoint:  \t' + self.endpoint + '\n'
		string = string + 'workingDir:\t' + self.workingdir + '\n'
		string = string + 'up:        \t' + self.compup + '\n'
		string = string + 'down:      \t' + self.compdown + '\n'
		string = string + 'x:         \t' + str(self.x) + '\n'
		string = string + 'y:         \t' + str(self.y) + '\n'
		string = string + 'Icon path: \t' + self.IconFilePath+"\n"
		return string	
	def setGraphicsData(self):#To set the graphicsItem datas
		self.graphicsItem.setIpColor()
		self.graphicsItem.setPos(self.x,self.y)
		self.graphicsItem.Alias=self.alias
	def changeStatus(self):#To change the color of status
class  ComponentTree(QtGui.QGraphicsView):	##The widget on which we are going to draw the graphtree 
	def __init__(self,parent,mainclass):
		QtGui.QGraphicsView.__init__(self,parent)
		self.mainclass=mainclass#This object is the mainClass from rcmanager Module
		self.CompoPopUpMenu=ComponentMenu(self)
		self.BackPopUpMenu=BackgroundMenu(self)
	def wheelEvent(self,wheel):
		#self.setTransformationAnchor(self.AnchorUnderMouse)
		temp=self.mainclass.currentZoom
		temp+=(wheel.delta()/120)
		self.mainclass.UI.verticalSlider.setValue(temp)
		self.mainclass.graph_zoom()
	def contextMenuEvent(self,event):##It will select what kind of context menu should be displayed
		pos=event.pos()
		item=self.itemAt(pos)
		if item:
			self.CompoPopUpMenu.setComponent(item)
			self.CompoPopUpMenu.popup(pos)
		else:
			self.BackPopUpMenu.popup(pos)



class ComponentScene(QtGui.QGraphicsScene):#The scene onwhich we are drawing the graph
	def __init__(self,arg=None):
		QtGui.QGraphicsScene.__init__(self)
		self.arg=arg
	


class ComponentMenu(QtGui.QMenu):
	def  __init__(self,parent):
		QtGui.QMenu.__init__(self,parent)
		self.ActionUp=QtGui.QAction("Up",parent)
		self.ActionDown=QtGui.QAction("Down",parent)
		self.ActionSettings=QtGui.QAction("Settings",parent)
		self.ActionControl=QtGui.QAction("Control",parent)
		self.ActionNewConnection=QtGui.QAction("New Connection",parent)
		self.addAction(self.ActionUp)
		self.addAction(self.ActionDown)
		self.addAction(self.ActionNewConnection)
		self.addAction(self.ActionControl)
		self.addAction(self.ActionSettings)
	def setComponent(self,component):
		self.currentComponent=component




class BackgroundMenu(QtGui.QMenu):
	def __init__(self,parent):
		QtGui.QMenu.__init__(self,parent)
		self.ActionSettings=QtGui.QAction("Settings",parent)
		self.ActionUp=QtGui.QAction("Up All",parent)
		self.ActionDown=QtGui.QAction("Down All",parent)
		self.ActionSearch=QtGui.QAction("Search",parent)
		self.ActionAdd=QtGui.QAction("Add",parent)
		self.addAction(self.ActionUp)
		self.addAction(self.ActionDown)
		self.addAction(self.ActionSettings)
		self.addAction(self.ActionAdd)
		self.addAction(self.ActionSearch)

def getDefaultValues():
	dict = {}
	dict['path'] =                   'kedit'
	dict['dock'] =                   'false'
	dict['fixed'] =                  1000               # Period of time in milliseconds between checks.
	dict['blink'] =                  300                # UI stuff. It the blinkin period (quite useless constant).
	dict['switch'] =                 2                  # Number of successive clicks to change a component state
	dict['interval'] =               400                # Maximum period of time in order to consider two clicks successive.
	dict['alpha'] =                  80
	dict['idletime'] =               1000               # 1000
	dict['focustime'] =              500                # 50
	dict['fasttime'] =               10                 # 10
	dict['fastperiod'] =             2000               # 2000
	dict['scale'] =                  200.               # 20.
	dict['hookes'] =                 0.07               # 0.02
	dict['springlength'] =           0.5               # 1.
	dict['friction'] =               0.4               # 0.2
	dict['step'] =                   0.5                # 1.
	dict['fieldforce'] =             20000.              # 0.4
	dict['active'] =                 'false'
	return dict

def getConfigFromFile(path):##This is the first function to be called for reading configurations for a xml file
	components = []	
	try:
		file = open(path, 'r')
	except:
		print 'Can\'t open ' + path + '.'

	data = file.read()
	xmldoc = libxml2.parseDoc(data)
	root = xmldoc.children
	
	if root is not None:
		components, NetworkSettings = parsercmanager(root)
	xmldoc.freeDoc()
	return components, NetworkSettings

def parsercmanager(node): #Call seperate functions for general settings and nodes
	components = []
	generalSettings=getDefaultValues()
	if node.type == "element" and node.name == "rcmanager":
		child = node.children
		while child is not None:
			if child.type == "element":
				if child.name == "generalInformation":
					parseGeneralInformation(child, generalSettings)
				elif child.name == "node":
					parseNode(child, components)
				elif stringIsUseful(str(node.properties)):
					print 'ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties)
			child = child.next
	return components, generalSettings

def parseGeneralInformation(node, dict): ##Takes care of reading the general information about the network tree
	if node.type == "element" and node.name == "generalInformation":
		child = node.children
		while child is not None:
			if child.type == "element":
				if child.name == "editor":
					parseEditor(child, dict)
				elif child.name == "timeouts":
					parseTimeouts(child, dict)
				elif child.name == "clicks":
					parseClicks(child, dict)
				elif child.name == "graph":
					parseGraph(child, dict)
				elif child.name == "graphTiming":
					parseGraphTiming(child, dict)
				elif child.name == "simulation":
					parseSimulation(child, dict)
				elif stringIsUseful(str(child.properties)):
					print 'ERROR when parsing generalInformation: '+str(child.name)+': '+str(child.properties)
			child = child.next
	elif node.type == "text":
		if stringIsUseful(str(node.properties)):
			print ''+str(node.properties)
	else:
		print "error: " + str(node.name)

def parseEditor(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['path', 'dock'])
	checkForMoreProperties(node)
def parseTimeouts(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['fixed', 'blink'])
	checkForMoreProperties(node)
	if dict.has_key('fixed'): dict['fixed'] = float(dict['fixed'])
	if dict.has_key('blink'): dict['blink'] = float(dict['blink'])
def parseClicks(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['switch', 'interval'])
	checkForMoreProperties(node)
	if dict.has_key('switch'): dict['switch'] = float(dict['switch'])
	if dict.has_key('interval'): dict['interval'] = float(dict['interval'])
def parseGraph(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['alpha', 'active', 'scale'])
	checkForMoreProperties(node)
	if dict.has_key('alpha'): dict['alpha'] = float(dict['alpha'])
	if dict.has_key('scale'): dict['scale'] = float(dict['scale'])
def parseGraphTiming(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['idletime', 'focustime', 'fasttime', 'fastperiod'])
	checkForMoreProperties(node)
	if dict.has_key('idletime'): dict['idletime'] = float(dict['idletime'])
	if dict.has_key('focustime'): dict['focustime'] = float(dict['focustime'])
	if dict.has_key('fasttime'): dict['fasttime'] = float(dict['fasttime'])
	if dict.has_key('fastperiod'): dict['fastperiod'] = float(dict['fastperiod'])
def parseSimulation(node, dict):##Called from parseGeneralInformation function
	parseGeneralValues(node, dict, ['hookes', 'springlength', 'friction', 'step', 'fieldforce'])
	checkForMoreProperties(node)
	if dict.has_key('hookes'): dict['hookes'] = float(dict['hookes'])
	if dict.has_key('springlength'): dict['springlength'] = float(dict['springlength'])
	if dict.has_key('friction'): dict['friction'] = float(dict['friction'])
	if dict.has_key('step'): dict['step'] = float(dict['step'])
	if dict.has_key('fieldforce'): dict['fieldforce'] = float(dict['fieldforce'])


def parseGeneralValues(node, dict, arg):##Called to read the attribute values of elements of General Values
	if node.children != None: print 'WARNING: No children expected'
	for attr in arg:
		if node.hasProp(attr):
			dict[attr] = node.prop(attr)
			node.unsetProp(attr)

def checkForMoreProperties(node):
	if node.properties != None: print 'WARNING: Attributes unexpected: ' + str(node.properties)


def parseNode(node, components):#To get the properties of a component
	if node.type == "element" and node.name == "node":
		child = node.children
		comp = CompInfo()
		comp.alias = parseSingleValue(node, 'alias', False)
		comp.endpoint = parseSingleValue(node, 'endpoint', False)
		mandatory = 0
		block_optional = 0
		while child is not None:
			if child.type == "element":
				if child.name == "workingDir":
					comp.workingdir = parseSingleValue(child, 'path')
					mandatory = mandatory + 1
				elif child.name == "upCommand":
					comp.compup = parseSingleValue(child, 'command')
					mandatory = mandatory + 2
				elif child.name == "downCommand":
					comp.compdown = parseSingleValue(child, 'command')
					mandatory = mandatory + 4
				elif child.name == "configFile":
					comp.configFile = parseSingleValue(child, 'path')
					mandatory = mandatory + 8
				elif child.name == "xpos":
					x=parseSingleValue(child, 'value')
					comp.x = float(x)
					block_optional = block_optional + 1
				elif child.name == "ypos":
					comp.y = float(parseSingleValue(child, 'value'))
					block_optional = block_optional + 2
				elif child.name == "dependence":
					comp.dependences.append(parseSingleValue(child, 'alias'))
				elif child.name == "icon":
					parseIcon(child, comp)
				elif child.name == "ip":
					comp.ip=parseSingleValue(child, "value")
				elif stringIsUseful(str(child.properties)):
					print 'ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties)
			child = child.next
		if mandatory<15:
			if   mandatory^15 == 1: print 'ERROR Not all mandatory labels were specified (workingDir)'
			elif mandatory^15 == 2: print 'ERROR Not all mandatory labels were specified (upCommand)'
			elif mandatory^15 == 4: print 'ERROR Not all mandatory labels were specified (downCommand)'
			elif mandatory^15 == 8: print 'ERROR Not all mandatory labels were specified (configFile)'
			raise NameError(mandatory)
		if block_optional<3 and block_optional != 0:
			if   block_optional^7 == 1: print 'ERROR Not all pos-radius labels were specified (xpos)'
			elif block_optional^7 == 2: print 'ERROR Not all pos-radius labels were specified (ypos)'
			raise NameError(block_optional)
		components.append(comp)
	elif node.type == "text":
		if stringIsUseful(str(node.properties)):
			print '    tssssssss'+str(node.properties)
	else:
		print "error: "+str(node.name)
	comp.setGraphicsData()
def parseSingleValue(node, arg, doCheck=True, optional=False):
	if node.children != None and doCheck == True: print 'WARNING: No children expected'+str(node)
	if not node.hasProp(arg) and not optional:
		print 'WARNING: ' + arg + ' attribute expected'
	else:
		ret = node.prop(arg)
		node.unsetProp(arg)
		return ret

def parseIcon(node, comp):
	x = parseSingleValue(node, 'value', optional=True)
	try:
		icon=QtGui.QPixmap(x)
		comp.graphicsItem.setIcon(icon)
		comp.IconFilePath=x
		if icon.isNull():
			raise NameError("Wrong file Path Given on item")
	except:
		print "Icon file path incorrect>>Icon set to default Value"     
		comp.IconFilePath="/home/h20/robocomp/tools/rcmanager/share/rcmanager/1465594390_sign-add.png" #THis is the default icon can be changed by users choice
		icon=QtGui.QPixmap(comp.IconFilePath)
		comp.graphicsItem.setIcon(icon)

def writeConfigToFile(dict, components, path):
	try:
		file = open(path, 'w')
	except:
		print 'Can\'t open ' + path + '.'
		return False
	writeToFile(file, '<?xml version="1.0" encoding="UTF-8"?>\n')
	writeToFile(file, '<rcmanager>\n')
	writeToFile(file, '\t<generalInformation>')


	if dict['path']!=None:
		writeToFile(file, '\t\t<editor path="'+str(dict['path'])+ '" dock="' + str(dict['dock'])  +'" />')
	if dict['fixed']!=None or dict['blink']!=None:
		string = '\t\t<timeouts '
		if dict['fixed']!=None: string = string + 'fixed="'+str(dict['fixed'])+'" '
		if dict['blink']!=None: string = string + 'blink="'+str(dict['blink'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['switch']!=None or dict['interval']!=None:
		string = '\t\t<clicks '
		if dict['switch']!=None: string = string + 'switch="'+str(dict['switch'])+'" '
		if dict['interval']!=None: string = string + 'interval="'+str(dict['interval'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['alpha']!=None or dict['active']!=None or dict['scale']!=None:
		string = '\t\t<graph '
		if dict['alpha']!=None: string = string + 'alpha="'+str(dict['alpha'])+'" '
		if dict['active']!=None: string = string + 'active="'+str(dict['active'])+'" '
		if dict['scale']!=None: string = string + 'scale="'+str(dict['scale'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['idletime']!=None or dict['focustime']!=None or dict['fasttime']!=None or dict['fastperiod']!=None:
		string = '\t\t<graphTiming '
		if dict['idletime']!=None: string = string + 'idletime="'+str(dict['idletime'])+'" '
		if dict['focustime']!=None: string = string + 'focustime="'+str(dict['focustime'])+'" '
		if dict['fasttime']!=None: string = string + 'fasttime="'+str(dict['fasttime'])+'" '
		if dict['fastperiod']!=None: string = string + 'fastperiod="'+str(dict['fastperiod'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['hookes']!=None or dict['springlength']!=None or dict['friction']!=None or dict['step']!=None or dict['fieldforce']!=None or dict['active']!=None:
		string = '\t\t<simulation '
		if dict['hookes']!=None: string = string + 'hookes="'+str(dict['hookes'])+'" '
		if dict['springlength']!=None: string = string + 'springlength="'+str(dict['springlength'])+'" '
		if dict['friction']!=None: string = string + 'friction="'+str(dict['friction'])+'" '
		if dict['step']!=None: string = string + 'step="'+str(dict['step'])+'" '
		if dict['fieldforce']!=None: string = string + 'fieldforce="'+str(dict['fieldforce'])+'" '
		string = string + '/>'
		writeToFile(file, string)

	writeToFile(file, '\t</generalInformation>')
 	writeToFile(file, '')

	for comp in components:
		comp.x=comp.graphicsItem.x()
		comp.y=comp.graphicsItem.y()
		writeToFile(file, '\t<node alias="' + comp.alias + '" endpoint="' + comp.endpoint + '">')
		for dep in comp.dependences:
			writeToFile(file, '  <dependence alias="' + dep + '" />')
		writeToFile(file, '\t\t<workingDir path="' + comp.workingdir + '" />')
		writeToFile(file, '\t\t<upCommand command="' + comp.compup + '" />')
		writeToFile(file, '\t\t<downCommand command="' + comp.compdown + '" />')
		writeToFile(file, '\t\t<configFile path="' + comp.configFile + '" />')
		writeToFile(file, '\t\t<xpos value="' + str(comp.x) + '" />')
		writeToFile(file, '\t\t<ypos value="' + str(comp.y) + '" />')
		writeToFile(file, '\t</node>\n')

	writeToFile(file, '</rcmanager>')

def writeToFile(file, string):#Write a line to the file
	file.write((string+'\n').encode( "utf-8" ))