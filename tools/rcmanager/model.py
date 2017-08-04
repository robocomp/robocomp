
from logger import RCManagerLogger
from yakuake_support import ProcessHandler
import xmlreader
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

    def load_from_xml(self, xml):
        # we go through the dictionary to create the graph
        # we have "rcmanager" and "nodes" keys

        # Clear previous data
        self.graph.clear()
        self.graph.node.clear()
        self.processId.clear()

        # Convert the xml data into python dict format
        xmldata = xmlreader.read_from_text(xml, 'xml')

        # creating nodes
        # the try catch block is added to handle cases
        # when the xml document contains no nodes
        try:
            for k in xmldata['rcmanager']['node']:
                self.add_node(k)
        except Exception, e:
            raise e

        # creating edges
        # the try catch block is added to handle cases
        # when the xml document contains no dependencies / edges
        try:
            for i in xmldata['rcmanager']['node']:
                if 'dependence' not in i.keys():
                    continue
                if len(i['dependence']) <= 1:
                    i['dependence'] = [i['dependence']]

                for j in i['dependence']:
                    self.add_edge(i['@alias'], j['@alias'])
        except Exception, e:
            raise e

        self.rcmanagerSignals.modelIsReady.emit()

    def add_node(self, nodedata):
        self.graph.add_node(nodedata['@alias'])
        self.processId[nodedata['@alias']] = -1
        for kk, vv in nodedata.items():
            self.graph.node[nodedata['@alias']][kk] = vv

    def add_edge(self, fromNode, toNode):
        self.graph.add_edge(fromNode, toNode)

    def get_component_running_status(self, componentAlias):
        if self.processId[componentAlias] == -1:
            return False
        elif not os.path.isdir("/proc/"+str(self.processId[componentAlias])):
            self.processId[componentAlias] = -1
            return False
        else:
            return True

    # this functions executes the command for starting a component
    def execute_start_command(self, componentAlias):
        try:
            if not self.get_component_running_status(componentAlias):
                tabTitle, processId = self.processHandler.start_process_in_existing_session(componentAlias, \
                                                                                            self.graph.node[componentAlias]['upCommand']['@command'])
                self.processId[componentAlias] = int(processId)
                self._logger.info("Component: " + componentAlias + " started in tab: " + tabTitle + " with PID: " + processId)
            else:
                self._logger.debug("Component: " + componentAlias + " is already running")
        except Exception, e:
            raise e
    
    # this functions executes the command for killing a component
    def execute_stop_command(self, componentAlias):
        if not self.get_component_running_status(componentAlias):
            self._logger.debug("Component: " + componentAlias + " is not running")
        else:
            self.processHandler.stop_process_in_session(componentAlias)
            self.processId[componentAlias] = -1
            self._logger.info("Component: " + componentAlias + " stopped")

    # this function writes the xml data to the given filename
    def export_xml_to_file(self, filename):
        open(filename, 'w').close()
        fileDescriptor = open(filename, 'a')

        line = '<?xml version="1.0" encoding="UTF-8"?>\n\n'
        line += '<rcmanager>\n'
        fileDescriptor.write(line + '\n')

        for i in self.graph.node.keys():
            line = self.dict_to_xml(self.graph.node[i], 'node', 1)
            fileDescriptor.write(line + '\n\n')

        line = '</rcmanager>'
        fileDescriptor.write(line + '\n')

        fileDescriptor.close()

    # this function converts the graph dict to the standard xml format used by rcmanager
    def dict_to_xml(self, dictionary, tagname, indentSpaceCount):
        line = (' ' * indentSpaceCount) + '<' + tagname

        subtags = list()

        for i in dictionary.keys():
            if not isinstance(dictionary[i], list):
                items = [dictionary[i]]
            else:
                items = dictionary[i]

            for j in items:
                if isinstance(j, dict):
                    subtags.append(self.dict_to_xml(j, i, (indentSpaceCount + 1)))
                else:
                    # eliminate the '@' symbol in the front
                    attr = i[:0] + i[(1):]
                    line = line + ' ' + attr + '="' + j + '"'

        if len(subtags) == 0:
            line = line + '/>'
        else:
            line = line + '>\n'
            for i in subtags:
                line = line + i + '\n'
            line = line + (' ' * indentSpaceCount) + '</' + tagname + '>'

        return line

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
