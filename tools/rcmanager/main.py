import sys, signal, argparse
from PyQt4 import QtCore, QtGui

def main():
	#read fle to dict
	#create model as a NetworkX graph using dict
	#create Qt Ui in a separate class
	#create controller



if __name__ == '__main__':
	# process params with a argparse
	app = QtGui.QApplication(sys.argv)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	main()
	ret = app.exec_()
	sys.exit(ret)


# window = MainClass()
#     window.show()
