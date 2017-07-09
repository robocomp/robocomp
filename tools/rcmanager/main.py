import sys, signal, argparse

from PyQt4.QtGui import QApplication
from xmlreader import xml_reader


def main():
	xmldata = xml_reader("manager.xml")
	#create model as a NetworkX graph using dict
	#create Qt Ui in a separate class
	#create controller



if __name__ == '__main__':
	# process params with a argparse
	app = QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	main()
	ret = app.exec_()
	sys.exit(ret)


# window = MainClass()
#     window.show()
