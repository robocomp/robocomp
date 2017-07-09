from PyQt4 import QtCore
import networkx as nx

class Model():
	"""docstring for Model"""

	def __init__(self, xmldata):
		print "------------------------------------"
		print "Hello, this is Model coming up"
		self.graph = nx.Graph()
		# we go through the dictionary to create the graph
		# we have "rcmanager" and "nodes" keys
		for k in xmldata["rcmanager"]["node"]:
			print "New node:", k['@alias']
			self.graph.add_node(k['@alias'])
			for kk, vv in k.items():
			#print kk, vv
				self.graph.node[k['@alias']][kk] = vv

		print "My new model graph:", list(self.graph.nodes(data=True))


