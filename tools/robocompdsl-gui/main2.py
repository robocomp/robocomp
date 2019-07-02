import sys
from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import QFile
from ui_gui import Ui_MainWindow



class RoboCompDSLGui(QMainWindow):
    def __init__(self):
        super(RoboCompDSLGui, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)



if __name__ == "__main__":
    app = QApplication(sys.argv)

    gui = RoboCompDSLGui()
    gui.show()

    sys.exit(app.exec_())
