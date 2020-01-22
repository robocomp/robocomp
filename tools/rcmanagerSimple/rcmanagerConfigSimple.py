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
import libxml2, sys
from PySide2 import QtCore, QtGui

filePath = 'rcmanager.xml'

#
# Component information container class.
#
class CompInfo:
	def __init__(self):
		self.endpoint = ''
		self.workingdir = ''
		self.compup = ''
		self.compdown = ''
		self.alias = ''
		self.dependences = []
		self.configFile = ''
		self.x = 0
		self.y = 0
		self.r = 10
		
	def __repr__(self):
		string = ''
		string = string + '[' + self.alias + ']:\n'
		string = string + 'endpoint:  \t' + self.endpoint + '\n'
		string = string + 'workingDir:\t' + self.workingdir + '\n'
		string = string + 'up:        \t' + self.compup + '\n'
		string = string + 'down:      \t' + self.compdown + '\n'
		string = string + 'x:         \t' + str(self.x) + '\n'
		string = string + 'y:         \t' + str(self.y) + '\n'
		string = string + 'r:         \t' + str(self.r) + '\n'
		return string

def unconnectedGroups(componentList):
	# Initialize the groups
	groups = []
	for listItem in componentList:
		l = []
		l.append(listItem.alias)
		groups.append(l)

	# Merge connected component groups
	cnt = True
	while cnt == True:
		cnt = False
		for group1 in groups:
			# Set the links of the group
			links = []
			for comp in group1:
				for c in componentList:
					if c.alias == comp:
						links = links + c.dependences
						break
			for dep in links:
				while links.count(dep) > 1: links.remove(dep)
			done = False
			for group2 in groups:
				if group2 == group1: continue
				for n in group2:
					if n in links:
						newGroup = []
						newGroup = newGroup + group1 + group2
						groups.remove(group1)
						groups.remove(group2)
						groups.append(newGroup)
						cnt = True
						break
				if cnt == True: break
			if cnt == True: break
	return len(groups)

def writeToFile(file, string):
	file.write((string+'\n').encode( "utf-8" ))

def writeConfigToFile(dict, components, path):
	try:
		file = open(path, 'w')
	except:
		print ('Can\'t open ' + path + '.')
		return False
	writeToFile(file, '<?xml version="1.0" encoding="UTF-8"?>\n')
	writeToFile(file, '<rcmanager>\n')
	writeToFile(file, ' <generalInformation>')


	if dict['path']!=None:
		writeToFile(file, '  <editor path="'+str(dict['path'])+ '" dock="' + str(dict['dock'])  +'" />')
	if dict['fixed']!=None or dict['blink']!=None:
		string = '  <timeouts '
		if dict['fixed']!=None: string = string + 'fixed="'+str(dict['fixed'])+'" '
		if dict['blink']!=None: string = string + 'blink="'+str(dict['blink'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['switch']!=None or dict['interval']!=None:
		string = '  <clicks '
		if dict['switch']!=None: string = string + 'switch="'+str(dict['switch'])+'" '
		if dict['interval']!=None: string = string + 'interval="'+str(dict['interval'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['alpha']!=None or dict['active']!=None or dict['scale']!=None:
		string = '  <graph '
		if dict['alpha']!=None: string = string + 'alpha="'+str(dict['alpha'])+'" '
		if dict['active']!=None: string = string + 'active="'+str(dict['active'])+'" '
		if dict['scale']!=None: string = string + 'scale="'+str(dict['scale'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['idletime']!=None or dict['focustime']!=None or dict['fasttime']!=None or dict['fastperiod']!=None:
		string = '  <graphTiming '
		if dict['idletime']!=None: string = string + 'idletime="'+str(dict['idletime'])+'" '
		if dict['focustime']!=None: string = string + 'focustime="'+str(dict['focustime'])+'" '
		if dict['fasttime']!=None: string = string + 'fasttime="'+str(dict['fasttime'])+'" '
		if dict['fastperiod']!=None: string = string + 'fastperiod="'+str(dict['fastperiod'])+'" '
		string = string + '/>'
		writeToFile(file, string)
	if dict['hookes']!=None or dict['springlength']!=None or dict['friction']!=None or dict['step']!=None or dict['fieldforce']!=None or dict['active']!=None:
		string = '  <simulation '
		if dict['hookes']!=None: string = string + 'hookes="'+str(dict['hookes'])+'" '
		if dict['springlength']!=None: string = string + 'springlength="'+str(dict['springlength'])+'" '
		if dict['friction']!=None: string = string + 'friction="'+str(dict['friction'])+'" '
		if dict['step']!=None: string = string + 'step="'+str(dict['step'])+'" '
		if dict['fieldforce']!=None: string = string + 'fieldforce="'+str(dict['fieldforce'])+'" '
		string = string + '/>'
		writeToFile(file, string)

	writeToFile(file, ' </generalInformation>')
	writeToFile(file, '')

	for comp in reversed(components):
		print (comp)
		writeToFile(file, ' <node alias="' + comp.alias + '" endpoint="' + comp.endpoint + '">')
		for dep in comp.dependences:
			writeToFile(file, '  <dependence alias="' + dep + '" />')
		writeToFile(file, '  <workingDir path="' + comp.workingdir + '" />')
		writeToFile(file, '  <upCommand command="' + comp.compup + '" />')
		writeToFile(file, '  <downCommand command="' + comp.compdown + '" />')
		writeToFile(file, '  <configFile path="' + comp.configFile + '" />')
		writeToFile(file, '  <xpos value="' + str(comp.x) + '" />')
		writeToFile(file, '  <ypos value="' + str(comp.y) + '" />')
		writeToFile(file, '  <radius value="' + str(comp.r)     + '" />')
		writeToFile(file, '  <color value="'  + str(comp.htmlcolor) + '" />')
		writeToFile(file, ' </node>\n')

	writeToFile(file, '</rcmanager>')


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
	# Spring-ForceField model
	dict['hookes'] =                 0.07               # 0.02
	dict['springlength'] =           0.5               # 1.
	dict['friction'] =               0.4               # 0.2
	dict['step'] =                   0.5                # 1.
	dict['fieldforce'] =             20000.              # 0.4
	dict['active'] =                 'false'
	return dict

def getConfigFromFile(path):
	components = []
	dict = getDefaultValues()

	# Abrimos el fichero de entrada
	try:
		file = open(path, 'r')
	except:
		print ('Can\'t open ' + path + '.')
		return [], getDefaultValues()

	data = file.read()
	xmldoc = libxml2.parseDoc(data)


	root = xmldoc.children
	if root is not None:
		components, newDict = parsercmanager(root)

	for k, v in newDict.items():
		dict[k] = v

	xmldoc.freeDoc()

	return components, dict

def parsercmanager(node):
	components = []
	dict = {}

	if node.type == "element" and node.name == "rcmanager":
		child = node.children
		while child is not None:
			if child.type == "element":
				if child.name == "generalInformation":
					parseGeneralInformation(child, dict)
				elif child.name == "node":
					parseNode(child, components)
				elif stringIsUseful(str(node.properties)):
					print ('ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties))
			child = child.next

	return components, dict

def parseGeneralInformation(node, dict):
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
					print ('ERROR when parsing generalInformation: '+str(child.name)+': '+str(child.properties))
			child = child.next
	elif node.type == "text":
		if stringIsUseful(str(node.properties)):
			print (''+str(node.properties))
	else:
		print ("error: " + str(node.name))


def parseNode(node, components):
	if node.type == "element" and node.name == "node":
		child = node.children
		component = CompInfo()
		component.alias = parseSingleValue(node, 'alias', False)
		component.endpoint = parseSingleValue(node, 'endpoint', False)
		component.color = None
		component.htmlcolor = None
		mandatory = 0
		block_optional = 0
		while child is not None:
			if child.type == "element":
				if child.name == "workingDir":
					parseWorkingDir(child, component)
					mandatory = mandatory + 1
				elif child.name == "upCommand":
					parseUpCommand(child, component)
					mandatory = mandatory + 2
				elif child.name == "downCommand":
					parseDownCommand(child, component)
					mandatory = mandatory + 4
				elif child.name == "configFile":
					parseConfigFile(child, component)
					mandatory = mandatory + 8
				elif child.name == "xpos":
					parseXPos(child, component)
					block_optional = block_optional + 1
				elif child.name == "ypos":
					parseYPos(child, component)
					block_optional = block_optional + 2
				elif child.name == "radius":
					parseRadius(child, component)
					block_optional = block_optional + 4
				elif child.name == "dependence":
					parseDependence(child, component)
				elif child.name == "color":
					parseColor(child, component)
				elif stringIsUseful(str(child.properties)):
					print ('ERROR when parsing rcmanager: '+str(child.name)+': '+str(child.properties))
			child = child.next
		if mandatory<15:
			if   mandatory^15 == 1: print ('ERROR Not all mandatory labels were specified (workingDir)')
			elif mandatory^15 == 2: print ('ERROR Not all mandatory labels were specified (upCommand)')
			elif mandatory^15 == 4: print ('ERROR Not all mandatory labels were specified (downCommand)')
			elif mandatory^15 == 8: print ('ERROR Not all mandatory labels were specified (configFile)')
			raise str(mandatory)
		if block_optional<7 and block_optional != 0:
			if   block_optional^7 == 1: print ('ERROR Not all pos-radius labels were specified (xpos)')
			elif block_optional^7 == 2: print ('ERROR Not all pos-radius labels were specified (ypos)')
			elif block_optional^7 == 4: print ('ERROR Not all pos-radius labels were specified (radius)')
			raise str(block_optional)
		components.append(component)
	elif node.type == "text":
		if stringIsUseful(str(node.properties)):
			print ('    tssssssss'+str(node.properties))
	else:
		print ("error: "+str(node.name))


def stringIsUseful(string):
	if len(string) == 0: return False
	if string[0] == '#': return False

	s1 = string
	s1 = s1.replace(' ', '')
	s1 = s1.replace('\t', '')
	s1 = s1.replace('\n', '')

	if len(s1) == 0: return False
	return True

def parseGeneralValues(node, dict, arg):
	if node.children != None: print ('WARNING: No children expected')
	for attr in arg:
		if node.hasProp(attr):
			dict[attr] = node.prop(attr)
			node.unsetProp(attr)

def checkForMoreProperties(node):
	if node.properties != None: print ('WARNING: Attributes unexpected: ' + str(node.properties))

#
# General information subfunctions
#
def parseEditor(node, dict):
	parseGeneralValues(node, dict, ['path', 'dock'])
	checkForMoreProperties(node)
def parseTimeouts(node, dict):
	parseGeneralValues(node, dict, ['fixed', 'blink'])
	checkForMoreProperties(node)
	if 'fixed' in dict: dict['fixed'] = float(dict['fixed'])
	if 'blink' in dict: dict['blink'] = float(dict['blink'])
def parseClicks(node, dict):
	parseGeneralValues(node, dict, ['switch', 'interval'])
	checkForMoreProperties(node)
	if 'switch' in dict: dict['switch'] = float(dict['switch'])
	if 'interval' in dict: dict['interval'] = float(dict['interval'])
def parseGraph(node, dict):
	parseGeneralValues(node, dict, ['alpha', 'active', 'scale'])
	checkForMoreProperties(node)
	if 'alpha' in dict: dict['alpha'] = float(dict['alpha'])
	if 'scale' in dict: dict['scale'] = float(dict['scale'])
def parseGraphTiming(node, dict):
	parseGeneralValues(node, dict, ['idletime', 'focustime', 'fasttime', 'fastperiod'])
	checkForMoreProperties(node)
	if 'idletime' in dict: dict['idletime'] = float(dict['idletime'])
	if 'focustime' in dict: dict['focustime'] = float(dict['focustime'])
	if 'fasttime' in dict: dict['fasttime'] = float(dict['fasttime'])
	if 'fastperiod' in dict: dict['fastperiod'] = float(dict['fastperiod'])
def parseSimulation(node, dict):
	parseGeneralValues(node, dict, ['hookes', 'springlength', 'friction', 'step', 'fieldforce'])
	checkForMoreProperties(node)
	if 'hookes' in dict: dict['hookes'] = float(dict['hookes'])
	if 'springlength' in dict: dict['springlength'] = float(dict['springlength'])
	if 'friction' in dict: dict['friction'] = float(dict['friction'])
	if 'step' in dict: dict['step'] = float(dict['step'])
	if 'fieldforce' in dict: dict['fieldforce'] = float(dict['fieldforce'])


#
# Node subfunctions
#
def parseSingleValue(node, arg, doCheck=True, optional=False):
	if node.children != None and doCheck == True: print ('WARNING: No children expected')
	if not node.hasProp(arg) and not optional:
		print ('WARNING: ' + arg + ' attribute expected')
	else:
		ret = node.prop(arg)
		node.unsetProp(arg)
		return ret


def parseDependence(node, comp):
	comp.dependences.append(parseSingleValue(node, 'alias'))
def parseColor(node, comp):
	comp.color = None
	x = parseSingleValue(node, 'value', optional=True)
	if x == None:
		return
	valid = True
	if len(x) != 7:
		valid = False
	else:
		if x[0] != "#": valid = False
		else:
			r = int(x[1:3], 16)
			g = int(x[3:5], 16)
			b = int(x[5:7], 16)
	if not valid:
		r = int(155)
		g = int(155)
		b = int(155)
	comp.color = QtGui.QColor(r,g,b)
	comp.htmlcolor = x
def parseWorkingDir(node, comp):
	comp.workingdir = parseSingleValue(node, 'path')
def parseUpCommand(node, comp):
	comp.compup = parseSingleValue(node, 'command')
def parseDownCommand(node, comp):
	comp.compdown = parseSingleValue(node, 'command')
def parseConfigFile(node, comp):
	comp.configFile = parseSingleValue(node, 'path')
def parseXPos(node, comp):
	comp.x = float(parseSingleValue(node, 'value'))
def parseYPos(node, comp):
	comp.y = float(parseSingleValue(node, 'value'))
def parseRadius(node, comp):
	comp.r = float(parseSingleValue(node, 'value'))



#if __name__ == "__main__":
	#c = []
	#d = {}
	#c, d = getConfigFromFile(filePath)
	#print '\nDict:\n'
	#print d
	#print '\nComponents\n'
	#print c
	#writeConfigToFile(d, c, '2.xml')


