
import sys, signal, argparse

from PyQt4.QtGui import QApplication
from xmlreader import xml_reader
from viewer import Viewer
from model import Model
from controller import Controller
from logger import RCManagerLogger
from PyQt4 import QtCore, QtGui
from rcmanagerSignals import rcmanagerSignals
import pdb

class Main():
    """This is the Main class which spawns the objects for the Model,
    Viewer and the Controller, for our MVC model."""

    def __init__(self):
        xmldata = xml_reader(sys.argv[1])
        
        self.signalObject = rcmanagerSignals()
        # pdb.set_trace()
		
        controller = Controller(self.signalObject)
        
        # create model as a NetworkX graph using dict
        self.model = Model(xmldata, self.signalObject)
        
        # create Qt Ui in a separate class
        self.viewer = Viewer(self.signalObject)
        self.viewer.show()
        
        # pass the viewer and model objects into the controller
        controller.viewer = self.viewer
        controller.model = self.model
        
        self.signalObject.init.emit('Controller')

if __name__ == '__main__':
    # process params with a argparse
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main = Main()
    ret = app.exec_()
    sys.exit(ret)


# window = MainClass()
# window.show()
