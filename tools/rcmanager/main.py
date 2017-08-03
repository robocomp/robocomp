
import sys, signal, argparse

from PyQt4.QtGui import QApplication
from xmlreader import xml_reader
from viewer import Viewer
from model import Model
from controller import Controller
from logger import RCManagerLogger
from PyQt4 import QtCore, QtGui
from rcmanagerSignals import RCManagerSignals
import argparse

class Main():
    """This is the Main class which spawns the objects for the Model,
    Viewer and the Controller, for our MVC model."""

    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("filename", help="the xml file containing the component graph data")
        args = parser.parse_args()
        self.signalObject = RCManagerSignals()
        
        # create model as a NetworkX graph using dict
        self.model = Model(self.signalObject)
        
        # create Qt Ui in a separate class
        self.viewer = Viewer(self.signalObject)
        self.viewer.show()
        
        # create a controller to connect the viewer and the model
        self.controller = Controller(self.model, self.viewer, self.signalObject)
        self.setup_signal_connection()

        self.signalObject.controllerIsReady.emit(sys.argv[1])
        
    def setup_signal_connection(self):
        self.signalObject.modelIsReady.connect(self.controller.model_init_action)
        self.signalObject.viewerIsReady.connect(self.controller.view_init_action)
        self.signalObject.controllerIsReady.connect(self.controller.controller_init_action)
        self.signalObject.saveModel.connect(self.controller.save_manager_file)
        self.signalObject.openModel.connect(self.controller.load_manager_file)
        self.signalObject.startComponent.connect(self.controller.start_component)
        self.signalObject.stopComponent.connect(self.controller.stop_component)

if __name__ == '__main__':
    # process params with a argparse
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main = Main()
    ret = app.exec_()
    sys.exit(ret)
    
# window = MainClass()
# window.show()
