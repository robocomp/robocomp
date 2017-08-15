
import threading
import xmlreader

from logger import RCManagerLogger
from PyQt4.QtGui import QColor

class Controller():
    """This is the Controller object for our MVC model. It connects the Model
    and the view, by reacting to the signals emitted by the view and
    making the necessary changes to the Model"""

    def __init__(self, model, view, rcmanagerSignals):
        self._logger = RCManagerLogger().get_logger("RCManager.Controller")

        self.need_to_save = False
        self.view = view
        self.model = model
        self.rcmanagerSignals = rcmanagerSignals

        self.isModelReady = False
        self.isViewReady = False
        self.isControllerReady = False

        self.signal_connections()

    def signal_connections(self):
        pass

    def model_init_action(self):
        self.isModelReady = True
        self._logger.info("Model object initialized")

    def view_init_action(self):
        self.isViewReady = True
        self._logger.info("View object initialized")

    def controller_init_action(self, filename, isNewFile=True):
        self.isControllerReady = True
        self._logger.info("Controller object initialized")

        # Save the filename for future use
        if isNewFile:
            self.view.filename = filename
            self.view.dirtyBit = False

        # Read the xml data from the file
        self.xml = xmlreader.get_text_from_file(str(filename))

        # Check the xml data for formatting issues
        if not xmlreader.validate_xml(self.xml):
            self._logger.error("XML validation failed. Please use a correctly formatted XML file")
            return

        self.model.load_from_xml(self.xml)
        self.load_model_into_viewer(self.xml)
        self.configure_viewer()

        self.view.check_component_status_thread = threading.Thread(target=self.model.check_component_status)
        self.view.check_component_status_thread.start()

    def start_component(self, componentAlias):
        self.model.execute_start_command(str(componentAlias))

    def stop_component(self, componentAlias):
        self.model.execute_stop_command(str(componentAlias))

    def load_model_into_viewer(self, xml):
        self.view.clear_graph_visualization()
        self.view.set_editor_text(xml)

        # adding nodes
        if self.view:
            for node, data in self.model.graph.nodes_iter(data=True):
                try:
                    xpos = float(data['xpos']['@value'])
                    ypos = float(data['ypos']['@value'])
                    position = (xpos, ypos)
                    self.view.add_node(node, data, position)
                except Exception, e:
                    self._logger.error("Node postion value for " + node + " are incorrect")
                    self.view.add_node(node, data)
            for orig, dest, data in self.model.graph.edges_iter(data=True):
                self.view.add_edge(orig, dest, data)
        else:
            raise Exception("A view must exist to update from model")

    def update_model(self):
        currentNodePosition = self.view.get_graph_nodes_positions()
        for i in currentNodePosition:
            xpos, ypos = currentNodePosition[i]

            new = {'@value': str(xpos)}
            old = self.model.graph.node[str(i)].get('xpos')
            if not old == new:
                self.view.dirtyBit = True
            self.model.graph.node[str(i)]['xpos'] = new

            new = {'@value': str(ypos)}
            old = self.model.graph.node[str(i)].get('ypos')
            if not old == new:
                self.view.dirtyBit = True
            self.model.graph.node[str(i)]['ypos'] = new

        color = self.view.graph_visualization.background_color
        new = {'@value': color.name()}
        old = self.model.generalInformation.get('backgroundColor')
        if not old == new:
            self.view.dirtyBit = True
        self.model.generalInformation['backgroundColor'] = {'@value': color.name()}

    def configure_viewer(self):
        if 'backgroundColor' in self.model.generalInformation.keys():
            color = QColor(self.model.generalInformation['backgroundColor']['@value'])
            self.view.set_background_color(color)

    def check_dirty_bit(self):
        index = self.view.tabWidget.currentIndex()
        if index == 0:
            self.update_model()
        elif index == 1:
            try:
                first = self.normalise_dict(xmlreader.read_from_text(str(self.xml), 'xml'))
                second = self.normalise_dict(xmlreader.read_from_text(str(self.view.codeEditor.text()), 'xml'))

                if not first == second:
                    self.view.dirtyBit = True
            except Exception, e:
                self._logger.error("XML file in code editor is incorrectly formatted")

    def normalise_dict(self, d):
        """
        Recursively convert dict-like object (eg OrderedDict) into plain dict.
        Sorts list values.
        """
        out = {}
        for k, v in dict(d).iteritems():
            if hasattr(v, 'iteritems'):
                out[k] = self.normalise_dict(v)
            elif isinstance(v, list):
                out[k] = []
                for item in sorted(v):
                    if hasattr(item, 'iteritems'):
                        out[k].append(self.normalise_dict(item))
                    else:
                        out[k].append(item)
            else:
                out[k] = v
        return out

    def load_manager_file(self, filename, isNewFile=True):
        self.controller_init_action(filename, isNewFile)

    def save_manager_file(self, filename):
        try:
            self.update_model()
            self.model.export_xml_to_file(str(filename))
            self.view.dirtyBit = False
        except Exception, e:
            self._logger.error("Couldn't save to file " + filename)
            raise e
