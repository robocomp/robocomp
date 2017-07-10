import sys, signal, argparse

from PyQt4.QtGui import QApplication
from xmlreader import xml_reader
from viewer import Viewer
from model import Model
from controller import Controller
import logging

from PyQt4 import QtCore, QtGui

# create main logger
logger = logging.getLogger('')
logger.setLevel(logging.DEBUG)
# create file handler which logs even debug messages
fh = logging.FileHandler('rcmanager.log')
fh.setLevel(logging.ERROR)
# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)

logger.info('creating an instance of auxiliary_module.Auxiliary')


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
