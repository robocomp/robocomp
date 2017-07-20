
from PyQt4 import QtCore
from xmlreader import xml_reader
import networkx as nx
import subprocess
import shlex

class Model():
	"""This is the Model object for our MVC model. It stores the component 
	graph and contains the functions needed to manipulate it."""
	
	def __init__(self, rcmanagerSignals=None):
		# print "------------------------------------"
		# print "Hello, this is Model coming up"
		
		# this is the class containing all of the custom signals used by rcmanager
		self.rcmanagerSignals = rcmanagerSignals
		
		# this is the networkx digraph object
		self.graph = nx.DiGraph()
		
		# this dictionary stores the process ids of all the started components
		# we store -1 for the components which are not running
		self.processId = dict()
			
	def load_from_xml(self, filename):
		# we go through the dictionary to create the graph
		# we have "rcmanager" and "nodes" keys
		
		xmldata = xml_reader(filename)
		
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
			
		self.rcmanagerSignals.modelIsReady.emit()
	
	def add_node(self, nodedata):
		self.graph.add_node(nodedata['@alias'])
		self.processId[nodedata['@alias']] = -1
		for kk, vv in nodedata.items():
			self.graph.node[nodedata['@alias']][kk] = vv

	def add_edge(self, fromNode, toNode):
		self.graph.add_edge(fromNode, toNode)
		
	# this functions executes the command for starting a component
	def up_component(self, component):
		try:
			if self.processId[component] == -1:
				proc = subprocess.Popen(shlex.split(self.graph.node[component]['upCommand']['@command']), shell=False)
				self.processId[component] = proc.pid
				print "Component:", component, "started with PID:", proc.pid
			else:
				print "Component:", component, "is already running"
		except Exception, e:
		    raise e
    
    # this functions executes the command for killing a component
    def down_component(self, component):
        if self.processId[component] == -1:
            print "Component:", component, "is not running"
        else:
            proc = subprocess.Popen(shlex.split("kill -9 "+str(self.processId[component])), shell=False)
            self.processId[component] = -1
            print "Component:", component, "stopped"

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

