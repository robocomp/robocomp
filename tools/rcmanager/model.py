
from PyQt4 import QtCore
import networkx as nx
# import pdb

class Model():
	"""This is the Model object for our MVC model. It stores the component 
	graph and contains the functions needed to manipulate it."""
	
	def __init__(self, xmldata=None, rcmanagerSignals=None):
		# print "------------------------------------"
		# print "Hello, this is Model coming up"
		
		self.rcmanagerSignals = rcmanagerSignals
		self.graph = nx.DiGraph()
		
		if xmldata == None:
			return
		
		# we go through the dictionary to create the graph
		# we have "rcmanager" and "nodes" keys
		
		# creating nodes
		# the try catch block is added to handle cases
		# when the xml document contains no nodes
		try:
			for k in xmldata["rcmanager"]["node"]:
				self.add_node(k)
		except:
			pass
			
		# creating edges
		# the try catch block is added to handle cases
		# when the xml document contains no dependencies / edges
		try:
			for i in xmldata["rcmanager"]["node"]:
				if len(i['dependence']) <= 1:
					i['dependence'] = [i['dependence']]
				
				for j in i["dependence"]:
					self.add_edge(i['@alias'], j['@alias'])
		except:
			pass
			
		self.rcmanagerSignals.init.emit("Model")
		
	def add_node(self, nodedata):
		self.graph.add_node(nodedata['@alias'])
		for kk, vv in nodedata.items():
			self.graph.node[nodedata['@alias']][kk] = vv

	def add_edge(self, fromNode, toNode):
		self.graph.add_edge(fromNode, toNode)
		
	# this functions emits a sample signal
	def sample_emit(self):
		print "sample signal was emitted"
		self.rcmanagerSignals.sample.emit('Model')
		
if __name__ == '__main__':
    # sample test case to see the working of the Model class  
    model = Model()
    
    model.add_node({'@alias': 'A'})
    model.add_node({'@alias': 'B'})
    model.add_node({'@alias': 'C'})
    model.add_node({'@alias': 'D'})
    
    model.add_edge('A', 'B')
    model.add_edge('B', 'C')
    model.add_edge('B', 'D')
    model.add_edge('C', 'D')
    model.add_edge('D', 'B')
    
    print "Number of nodes:", model.graph.number_of_nodes()
    print "Number of edges:", model.graph.number_of_edges()
    print "Adjacencies: ", model.graph.adj
	
