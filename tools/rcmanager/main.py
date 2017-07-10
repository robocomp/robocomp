import sys, signal, argparse

from PyQt4.QtGui import QApplication
from xmlreader import xml_reader
from viewer import Viewer
from model import Model
from controller import Controller
from logger import RCManagerLogger

from PyQt4 import QtCore, QtGui


class Main():
    """docstring for MainClass"""

    def __init__(self):
        xmldata = xml_reader("manager.xml", False)
        # create model as a NetworkX graph using dict
        self.model = Model(xmldata)
        # create Qt Ui in a separate class
        self.viewer = Viewer()
        self.viewer.show()
        # create controller
        controller = Controller(self.model, self.viewer)


if __name__ == '__main__':
    # process params with a argparse
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main = Main()
    ret = app.exec_()
    sys.exit(ret)


# window = MainClass()
#     window.show()
