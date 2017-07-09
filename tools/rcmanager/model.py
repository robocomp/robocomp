
from PyQt4 import QtCore
import networkx as nx
import pdb

class Node():
	def __init__(self, args=None):
		self.alias = None
		self.endpoint = None
		self.group = None
		self.groupName = None
		self.xpos = 0
		self.ypos = 0
		self.upCommand = None
		self.downCommand = None
		self.workingdir = None
		self.configFile = None
		
		self.asEnd = []  # This is the list of connection where the node act as the ending point
		self.asBeg = []  # This is the list of connection where the node act as the beginning point
		self.dependences = []
		
		self.ip = None
		self.iconFilePath = None
		self.status = False
		self.nodeColor = [0, 0, 0]

class Model():
	"""docstring for Model"""
	def __init__(self, jsonobject):
		print "------------------------------------"
		print "Hello, this is Model coming up"
		
		self.graph = nx.DiGraph()
		
		# we go through the dictionary to create the graph
		# we have "rcmanager" and "nodes" keys
		
		# creating nodes
		# the try catch block is added to handle cases
		# when the xml document contains no nodes
		try:
			for i in jsonobject["rcmanager"]["node"]:
				node = self.get_node_from_json(i)
				self.add_node(i['@alias'], node)
				#pdb.set_trace()
		except:
			pass
				
		# creating edges
		# the try catch block is added to handle cases
		# when the xml document contains no dependencies / edges
		try:
			for i in jsonobject["rcmanager"]["node"]:
				if len(i['dependence']) <= 1:
					i['dependence'] = [i['dependence']]
				
				for j in i["dependence"]:
					self.add_edge(i['@alias'], j['@alias'])
		except:
			pass
			
		print "My new model graph:", list(self.graph.nodes(data=True))
		
	def get_node_from_json(self, jsonobject):
		node = Node()
		node.alias = jsonobject['@alias']
		node.endpoint = jsonobject['@endpoint']
		node.xpos = jsonobject['xpos']['@value']
		node.ypos = jsonobject['ypos']['@value']
		node.upCommand = jsonobject['upCommand']['@command']
		node.downCommand = jsonobject['downCommand']['@command']
		node.workingDir = jsonobject['workingDir']['@path']
		node.configFile = jsonobject['configFile']['@path']
		
		print "Added node:", node.alias
		return node

	def add_node(self, alias=None, node=None):
		self.graph.add_node(alias)
		self.graph.node[alias] = node
		
	def add_edge(self, fromNode=None, toNode=None):
		self.graph.add_edge(fromNode, toNode)
	
		print "Added edge from:", fromNode, "to:", toNode
