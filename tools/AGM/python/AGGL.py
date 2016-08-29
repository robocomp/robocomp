import math, traceback, itertools, copy, sys, inspect

from pddlAGGL import *

def distance(x1, y1, x2, y2):
	return math.sqrt(math.pow(x1-x2, 2) + math.pow(y1-y2, 2))

## AGM Symbol
# @ingroup PyAPI
#
# @param name The identifier of the symbol
# @param sType The type of the symbol
# @param attributes An optional attribute map
class AGMSymbol(object):
	def __init__(self, name, sType, pos=None, attributes=None):
		object.__init__(self)
		if pos==None: pos=[0,0]
		if attributes==None: attributes=dict()
		self.name = str(name)
		self.sType = str(sType)
		self.pos = pos
		self.color = 'white'
		self.attributes = attributes
	def __str__(self):
		return self.toString()
	def __repr__(self):
		return self.toString()
	def __eq__(self, other):
		if self.sType == other.sType and self.name == other.name:
			return True
		else:
			return False
	def equivalent(self, other):
		if self.sType == other.sType and self.name == other.name:
			return True
		else:
			return False
	def __cmp__(self):
		return object.__cmp__(self)
	def toString(self):
		return str(self.name)+':'+str(self.sType)
	@property
	def x(self):
		return self.pos[0]
	@property
	def y(self):
		return self.pos[1]
	def linkedTo(self, node, graph):
		for link in graph.links:
			if link.a == self.name and link.b == node.name:
				return True
		return False


## AGM Link
# @ingroup PyAPI
#
class AGMLink(object):
	def __init__(self, a, b, linkType, attrs=None, enabled=True):
		object.__init__(self)
		self.a = a
		self.b = b
		self.linkType = linkType
		self.attributes = attrs

		if self.attributes == None:
			self.attributes = {}
		elif type(self.attributes) == type({}):
			self.attributes = attrs
		else:
			print 'Wrong value for AGMLink() [attrs is not a dictionary]', type(attrs)
			sys.exit(-1358)

		if linkType == 'RT':
			if not 'tx' in self.attributes: self.attributes['tx'] = '0'
			if not 'ty' in self.attributes: self.attributes['ty'] = '0'
			if not 'tz' in self.attributes: self.attributes['tz'] = '0'
			if not 'rx' in self.attributes: self.attributes['rx'] = '0'
			if not 'ry' in self.attributes: self.attributes['ry'] = '0'
			if not 'rz' in self.attributes: self.attributes['rz'] = '0'
		
			
		self.color = 'white'
		self.enabled = enabled
	## Converts an AGMLink to a string
	# @retval A string representing the link
	def toString(self):
		v = ''
		if self.enabled:
			return '\t\t'+str(self.a)+'->'+str(self.b)+'('+v+str(self.linkType)+')'
		else:
			return '\t\t'+str(self.a)+'->'+str(self.b)+'('+v+str(self.linkType)+')*'
	def __cmp__(self, other):
		this =  self.linkType+" "+ self.a+" "+ self.b+" "+str( self.enabled)
		if isinstance(other, list):
			nhis = other[2]+" "+other[0]+" "+other[1]+" "
			if len(other)>3:
				nhis+=str(other[3])
			else:
				nhis+=str(True)
		else:
			nhis = other.linkType+" "+other.a+" "+other.b+" "+str(other.enabled)
		if this < nhis: return -1
		if this > nhis: return +1
		return 0
	def __eq__(self, other):
		#this = self.linkType+" "+ self.a+" "+ self.b+" "+str( self.enabled)
		if isinstance(other, list):
			if other[2] != self.linkType: return False
			if other[0] != self.a: return False
			if other[1] != self.b: return False
			if len(other)>3:
				if str(self.enabled) != str(other[3]): return False
			else:
				if str(self.enabled) != str(True): return False
		else:
			if self.a != other.a: return False
			if self.b != other.b: return False
			if self.linkType != other.linkType: return False
			if self.enabled != other.enabled: return False
		return True
	def __ne__(self, other):
		this =  self.linkType+" "+ self.a+" "+ self.b+" "+str( self.enabled)
		if isinstance(other, list):
			nhis = other[2]+" "+other[0]+" "+other[1]+" "
			if len(other)>3:
				nhis+=str(other[3])
			else:
				nhis+=str(True)
		else:
			nhis = str(other.linkType) + " " + str(other.a) + " " + str(other.b) + " " + str(other.enabled)
		if this == nhis:
			return False
		return True
	def __hash__(self):
		return len(self.a+self.b+self.linkType+str(self.enabled))
	def __str__(self):
		ret = '('+str(self.a)+')--['+str(self.linkType)+']--('+str(self.b)+') '
		if not self.enabled:
			ret += '*'
		return ret
	def __repr__(self):
		return self.__str__()

## AGM Graph
# @ingroup PyAPI
#
class AGMGraph(object):
	def __init__(self, nodes=None, links=None, side='n'):
		object.__init__(self)
		if links == None:
			links = list()
		if nodes == None:
			nodes = dict()
		self.nodes = nodes
		self.links = links
		self.side = side
		self.version = 0

	def getIsolatedSymbolsNames(self):
		connected = set()
		for link in self.links:
			connected.add(link.a)
			connected.add(link.b)
		return [ n for n in self.nodes.keys() if not n in connected ]

	def filterGeometricSymbols(self):
		# Create a deep copy of the current graph
		ret = copy.deepcopy(self)
		# Remove RT links
		ret.links = [ l for l in ret.links if not l.linkType == "RT"]
		# Get isolated nodes
		isolated = ret.getIsolatedSymbolsNames()
		# Remove isolated nodes
		for n in isolated:
			del ret.nodes[n]
		# Return resulting graph
		return ret
	def removeDanglingEdges(self):
		linkindex = 0
		while linkindex < len(self.links):
			e = self.links[linkindex]
			try:
				v1 = self.nodes[e.a]
				v2 = self.nodes[e.b]
				linkindex += 1
			except:
				del self.links[linkindex]
	def __str__(self):
		ret = 'NODES [ '
		for i in self.nodes:
			ret += ' '+str(self.nodes[i])+' '
		ret += ']\n'
		ret += 'LINKS [ '
		for l in self.links:
			ret += ' '+str(l)+' '
		ret += ']\n'
		return ret

	def __repr__(self):
		return self.__str__()
	def __cmp__(self):
		print 'AGMGraph cmp(self)'
		return object.__cmp__(self)
	def __hash__(self):
		return (len(self.nodes), len(self.links))
	def __eq__(self, other):
		try:
			# Basic: number of nodes
			if len(self.nodes) != len(other.nodes):
				return False
			# Basic: number of links
			if len(self.links) != len(other.links):
				return False
			# Exhaustive nodes
			for l in self.nodes:
				if not self.nodes[l].__eq__(other.nodes[l]):
					return False
			for l in range(len(self.links)):
				if not self.links[l] in other.links:
					return False
			return True
		except:
			return False

	def equivalent(self, other):
		try:
			# Basic: number of nodes
			if len(self.nodes) != len(other.nodes):
				return False
			# Basic: number of links
			if len(self.links) != len(other.links):
				return False
			# Exhaustive nodes
			for l in self.nodes:
				if not self.nodes[l].equivalent(other.nodes[l]):
					return False
			for l in range(len(self.links)):
				if not self.links[l] in other.links:
					return False
			return True
		except:
			return False

	def __cmp__(self, other):
		#print 'AGMGraph cmp(self, other)'
		# Basic: number of nodes
		if len(self.nodes) != len(other.nodes):
			return -1
		# Basic: number of links
		if len(self.links) != len(other.links):
			return -1

		return 0
	def getName(self, xa, ya, diameter):
		minDist = -1
		minName = None
		for name in self.nodes.keys():
			dist = distance(xa, ya, self.nodes[name].pos[0], self.nodes[name].pos[1])
			if dist < diameter/2:
				if dist < minDist or minDist < 0:
					minDist = dist
					minName = name
		if minDist > -1:
			return minName, True
		else:
			raise Exception("")
	def setColors(self, other, left):
		for name in self.nodes.keys():
			if not name in other.nodes.keys():
				if left:
					self.nodes[name].color = "red"
				else:
					self.nodes[name].color = "green"
			else:
				if self.nodes[name].sType != other.nodes[name].sType:
					self.nodes[name].color = "gray"
				else:
					self.nodes[name].color = "white"
		for l in range(len(self.links)):
			if not self.links[l] in other.links:
				if left:
					self.links[l].color = "red"
				else:
					self.links[l].color = "green"
			else:
				self.links[l].color = "white"
	def getNodeChanges(self, other, parameters):
		# Generate temporal variables
		parametersDict = dict()
		#if len(parameters.strip()) > 0:
		for i in parameters:
			parametersDict[i[0]] = AGMSymbol(i[0], i[1])
		L = dict( self.nodes, **parametersDict)
		R = dict(other.nodes, **parametersDict)
		# Initialize return values
		toCreate = []
		toRemove = []
		toRetype = []
		# Find symbols to create and retype
		for name in L.keys():
			if not name in R.keys():
					toRemove.append(L[name])
			else:
				if L[name].sType != R[name].sType:
					toRetype.append(L[name])
		# Find symbols to remove
		for name in R.keys():
			if not name in L.keys():
					toCreate.append(R[name])
		return toCreate, toRemove, toRetype
	def getLinkChanges(self, other):
		toCreate = []
		toRemove = []
		for l in range(len(self.links)):
			if not self.links[l] in other.links:
				toRemove.append(self.links[l])
		for l in range(len(other.links)):
			if not other.links[l] in self.links:
				toCreate.append(other.links[l])
		return toCreate, toRemove
	def getNameRelaxed(self, xa, ya, diameter):
		minDist = -1
		minName = None
		for name in self.nodes.keys():
			dist = distance(xa, ya, self.nodes[name].pos[0], self.nodes[name].pos[1])
			if dist < minDist or minDist < 0:
				minDist = dist
				minName = name
		if minDist > -1:
			return minName, True
		else:
			return '', False
	def getCenter(self, xa, ya, diameter):
		name, found = self.getName(xa, ya, diameter)
		if found:
			return self.nodes[name].pos[0], self.nodes[name].pos[1]
		else:
			raise BasicException("")


	def addNode(self, x, y, name, stype, attributes=None):
		if attributes==None: attributes=dict()
		if not name in self.nodes.keys():
			self.nodes[name] = AGMSymbol(str(name), str(stype), [x,y], attributes)

	def removeNode(self, x, y, diameter):
		name, found = self.getName(x, y, diameter)
		if found:
			del self.nodes[name]
			self.removeEdgesRelatedTo(name)
		else:
			pass

	def removeNodeByName(self, name):
		del self.nodes[name]
		self.removeEdgesRelatedTo(name)

	def removeEdgesRelatedTo(self, name):
		for link in self.links:
			if link.a == name or link.b == name:
				self.removeEdge(link.a, link.b)


	def moveNode(self, name, x, y, diameter):
		if name in self.nodes:
			self.nodes[name].pos = [x, y]
		elif name == '':
			pass
		else:
			print 'No such node', str(name)+'. Internal editor error.'
			pass
	def addEdge(self, a, b, linkname='link',attrs=None):
		if attrs==None: attrs=dict()
		self.links.append(AGMLink(a, b, linkname, attrs, enabled=True))
	def removeEdge(self, a, b):
		i = 0
		while i < len(self.links):
			l = self.links[i]
			if l.a == a and l.b == b:
				del self.links[i]
			else:
				i += 1
	def toString(self):
		ret = '\t{\n'
		for v in self.nodes.keys():
			ret += '\t\t' +str(self.nodes[v].name) + ':'+self.nodes[v].sType + '(' + str(int(self.nodes[v].pos[0])) + ','+ str(int(self.nodes[v].pos[1])) + ')\n'
		for l in self.links:
			ret += l.toString() + '\n'
		return ret+'\t}'

	def nodeTypes(self):
		ret = set()
		for k in self.nodes.keys():
			ret.add(self.nodes[k].sType)
		return ret

	def nodeNames(self):
		ret = set()
		for k in self.nodes.keys():
			ret.add(self.nodes[k].name)
		return ret

	def linkTypes(self):
		ret = set()
		for l in self.links:
			ret.add(l.linkType)
		return ret

	def getNode(self, identifier):
		return self.nodes[identifier]

	def toXML(self, path):
		f = open(path, 'w')
		f.write('<AGMModel>\n')
		for n in self.nodes:
			f.write('\t<symbol id="'+str(self.nodes[n].name)+'" type="'+str(self.nodes[n].sType)+'">\n')
			for attr in self.nodes[n].attributes.keys():
				f.write('\t\t<attribute key="'+attr+'" value="'+self.nodes[n].attributes[attr]+'" />\n')
			f.write('\t</symbol>\n')
		for l in self.links:
			f.write('\t<link src="'+str(l.a)+'" dst="'+str(l.b)+'" label="'+str(l.linkType)+'" >\n')
			for attr in l.attributes.keys():
				f.write('\t\t<linkAttribute key="'+attr+'" value="'+l.attributes[attr]+'" />\n')
			f.write('\t</link>\n')
		f.write('</AGMModel>\n\n')
		f.close()


## AGM Rule
# @ingroup PyAPI
#
class AGMRule(object):
	def __init__(self, name='', lhs=None, rhs=None, passive=False, cost=1, success=1., parameters='', precondition='', effect='', dormant=False, activates=None):
		object.__init__(self)
		self.name = name
		self.lhs = lhs
		self.rhs = rhs
		self.cost = cost
		if len(str(success)) == 0:
			self.success = 1.
		else:
			self.success = float(str(success))
		self.dormant = dormant
		if activates == None:
			activates = []
		self.activates = activates
		self.passive = passive
		self.parameters = parameters
		self.precondition = precondition
		self.effect = effect
		if lhs == None:
			self.lhs = AGMGraph()
		if rhs == None:
			self.rhs = AGMGraph()
	def toString(self):
		passiveStr = "active"
		if self.passive: passiveStr = "passive"
		costStr = str(self.cost)
		ret = self.name + ' : ' + passiveStr + '(' + costStr + ')'
		ret += '\n{\n'
		ret += self.lhs.toString() + '\n'
		ret += '\t=>\n'
		ret += self.rhs.toString() + '\n'
		if len(self.parameters) > 0:
			ret += '\tparameters\n\t{' + self.parameter + '}\n'
		if len(self.precondition) > 0:
			ret += '\tprecondition\n\t{' + self.precondition + '}\n'
		if len(self.effect) > 0:
			ret += '\teffect\n\t{' + self.effect + '}\n'
		ret += '}'
		return ret
	def forgetNodesList(self):
		return list(set(self.lhs.nodes).difference(set(self.rhs.nodes)))
	def newNodesList(self):
		return list(set(self.rhs.nodes).difference(set(self.lhs.nodes)))
	def stayingNodeSet(self):
		return set(self.lhs.nodeNames()).intersection(set(self.rhs.nodeNames()))
	def stayingNodeList(self):
		return list(self.stayingNodeSet())
	def anyNewOrForgotten(self):
		if len(self.newNodesList())==0:
			if len(self.newNodesList())==0:
				return False
		return True
	def nodeTypes(self):
		return self.lhs.nodeTypes().union(self.rhs.nodeTypes())
	def nodeNames(self):
		return self.lhs.nodeNames().union(self.rhs.nodeNames())
	def linkTypes(self):
		return self.lhs.linkTypes().union(self.rhs.linkTypes())




## AGM Combo rule
# @ingroup PyAPI
#
class AGMComboRule(object):
	def __init__(self, name='', passive=False, cost=1, ats=None, eqs=None):
		object.__init__(self)
		if ats==None: ats=list()
		if eqs==None: eqs=list()
		self.name = name
		self.passive = passive
		self.atoms = ats
		self.cost = cost
		self.equivalences = []
		for eq in eqs:
			eqResult = list()
			for element in eq:
				eqResult.append([element[0], element[1]])
			self.equivalences.append(eqResult)
		self.text = self.generateTextFromCombo()
	def generateTextFromCombo(self):
		ret = ''
		for a in self.atoms:
			ret += '\t' + a[0] + ' as ' + a[1] + '\n'
		ret += '\twhere:\n'
		for e in self.equivalences:
			first = True
			for element in e:
				if first:
					first = False
					ret += '\t'  + element[0] + '.' + element[1]
				else:
					ret += ' = ' + element[0] + '.' + element[1]
			ret += '\n'
		return ret

	def toString(self):
		passiveStr = "active"
		if self.passive: passiveStr = "passive"
		ret = self.name + ' : ' + passiveStr + '('+ str(self.cost) +')\n{\n'

		if len(self.text) > 0:
			ret += self.text
		else:
			for a in self.atoms:
				ret += '\t' + a[0] + ' as ' + a[1] + '\n'
			ret += '\twhere:\n'
			for e in self.equivalences:
				first = True
				for element in e:
					if first:
						first = False
						ret += '\t'  + element[0] + '.' + element[1]
					else:
						ret += ' = ' + element[0] + '.' + element[1]
				ret += '\n'

		ret += '}\n'
		return ret



## AGM Hierarchical rule
# @ingroup PyAPI
#
class AGMHierarchicalRule(object):
	def __init__(self, name='', lhs=None, rhs=None, passive=False, cost=1, success=1., dormant=False, activates=None):
		object.__init__(self)
		self.cost = cost
		if len(str(success)) == 0:
			self.success = 1.
		else:
			self.success = float(str(success))
		self.lhs = lhs
		if lhs == None: self.lhs = AGMGraph()
		self.rhs = rhs
		if rhs == None: self.rhs = AGMGraph()
		self.name = name
		self.dormant = dormant
		if activates == None:
			activates = []
		self.activates = activates
		self.passive = passive
		self.text = self.generateTextFromHierarchical()
	def generateTextFromHierarchical(self):
		ret = ''
		ret += self.lhs.toString() + '\n'
		ret += '\t=>\n'
		ret += self.rhs.toString() + '\n'
	def toString(self):
		passiveStr = "active"
		if self.passive: passiveStr = "passive"
		ret = self.name + ' : ' + passiveStr + '('+ str(self.cost) +')\n{\n'

		if len(self.text) > 0:
			ret += self.text
		else:
			ret += self.lhs.toString() + '\n'
			ret += '\t=>\n'
			ret += self.rhs.toString() + '\n'
		ret += '}\n'
		return ret



## AGM class
# @ingroup PyAPI
#
class AGM(object):
	def __init__(self):
		object.__init__(self)
		self.rules = []
	def addRule(self, rule):
		self.rules.append(rule)
	def getInitiallyAwakeRules(self):
		ret = set()
		for rule in self.rules:
			if not rule.dormant:
				ret.add(str(rule.name))
		return ret

## AGM file data
# @ingroup PyAPI
#
class AGMFileData(object):
	def __init__(self):
		object.__init__(self)
		self.properties = dict()
		self.agm = AGM()

	def addRule(self, rule):
		self.agm.addRule(rule)

	def getInitiallyAwakeRules(self):
		return self.agm.getInitiallyAwakeRules()

	def toFile(self, filename):
		writeString = ''
		for k,v in self.properties.items():
			writeString += str(k) + '=' + str(v) + '\n'
		writeString += '===\n'
		# Rules
		for r in self.agm.rules:
			writeString = writeString + r.toString() + '\n\n'
		w = open(filename, 'w')
		w.write(writeString)
		w.close()

	## Generates PDDL code
	# @param filename The path of the file where the PDDL version of the grammar is written
	# @param skipPassiveRules Flag used to generate only active rules (false by default)
	def generatePDDL(self, filename, skipPassiveRules=False):
		#print 'Generating (partial =', str(skipPassiveRules)+') PDDL file'
		w = open(filename, 'w')
		a = copy.deepcopy(self.agm)
		text = AGMPDDL.toPDDL(a, self.properties["name"], skipPassiveRules)
		w.write(text)
		w.close()

	## Generates AGGL Planner code
	# @param filename The path of the file where the python version of the grammar is written
	# @param skipPassiveRules Flag used to generate only active rules (false by default)
	def generateAGGLPlannerCode(self, filename, skipPassiveRules=False):
		print 'generateAGGLPlannerCode'
		w = open(filename, 'w')
		a = copy.deepcopy(self.agm)
		import generateAGGLPlannerCode
		text = generateAGGLPlannerCode.generate(a, skipPassiveRules)
		w.write(text)
		w.close()

