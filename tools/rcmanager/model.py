
from logger import RCManagerLogger
from yakuake_support import ProcessHandler
from xmlreader import xml_reader
import networkx as nx
import os

class Model():
    """This is the Model object for our MVC model. It stores the component
    graph and contains the functions needed to manipulate it."""

    def __init__(self, rcmanagerSignals=None):
        self._logger = RCManagerLogger().get_logger("RCManager.Model")

        # this is the class containing all of the custom signals used by rcmanager
        self.rcmanagerSignals = rcmanagerSignals

        # this is the networkx digraph object
        self.graph = nx.DiGraph()

        # this dictionary stores the process ids of all the started components
        # we store -1 for the components which are not running
        self.processId = dict()

        # this is the process handler for the model
        self.processHandler = ProcessHandler()

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
        if not os.path.isdir("/proc/"+str(self.processId[component])):
            self.processId[component] = -1

        try:
            if self.processId[component] == -1:
                tabTitle, processId = self.processHandler.start_process_in_existing_session(component, \
                                                                    self.graph.node[component]['upCommand']['@command'])
                self.processId[component] = int(processId)
                self._logger.info("Component: " + component + " started in tab: " + tabTitle + " with PID: " + processId)
            else:
                self._logger.debug("Component: " + component + " is already running")
        except Exception, e:
            raise e
    
    # this functions executes the command for killing a component
    def down_component(self, component):
        if not os.path.isdir("/proc/"+str(self.processId[component])):
            self.processId[component] = -1

        if self.processId[component] == -1:
            self._logger.debug("Component: " + component + " is not running")
        else:
            self.processHandler.stop_process_in_session(component)
            self.processId[component] = -1
            self._logger.info("Component: " + component + " stopped")

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
