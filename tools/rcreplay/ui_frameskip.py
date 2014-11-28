# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'frameskip.ui'
#
# Created: Wed Nov 26 22:19:36 2014
#      by: PyQt4 UI code generator 4.11.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_ReplayFrameskipMainWindow(object):
    def setupUi(self, ReplayFrameskipMainWindow):
        ReplayFrameskipMainWindow.setObjectName(_fromUtf8("ReplayFrameskipMainWindow"))
        ReplayFrameskipMainWindow.resize(585, 303)
        self.centralwidget = QtGui.QWidget(ReplayFrameskipMainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.tab)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label = QtGui.QLabel(self.tab)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.inputFile = QtGui.QLineEdit(self.tab)
        self.inputFile.setObjectName(_fromUtf8("inputFile"))
        self.horizontalLayout.addWidget(self.inputFile)
        self.toolButton = QtGui.QToolButton(self.tab)
        self.toolButton.setObjectName(_fromUtf8("toolButton"))
        self.horizontalLayout.addWidget(self.toolButton)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.label_2 = QtGui.QLabel(self.tab)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout.addWidget(self.label_2)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.outputFile = QtGui.QLineEdit(self.tab)
        self.outputFile.setObjectName(_fromUtf8("outputFile"))
        self.horizontalLayout_2.addWidget(self.outputFile)
        self.toolButton_2 = QtGui.QToolButton(self.tab)
        self.toolButton_2.setObjectName(_fromUtf8("toolButton_2"))
        self.horizontalLayout_2.addWidget(self.toolButton_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.label_3 = QtGui.QLabel(self.tab)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout.addWidget(self.label_3)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem)
        self.mps = QtGui.QSpinBox(self.tab)
        self.mps.setMinimumSize(QtCore.QSize(200, 0))
        self.mps.setMinimum(1)
        self.mps.setMaximum(50)
        self.mps.setObjectName(_fromUtf8("mps"))
        self.horizontalLayout_5.addWidget(self.mps)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem1)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        spacerItem2 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem2)
        self.fileButton = QtGui.QPushButton(self.tab)
        self.fileButton.setObjectName(_fromUtf8("fileButton"))
        self.horizontalLayout_3.addWidget(self.fileButton)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_3 = QtGui.QWidget()
        self.tab_3.setObjectName(_fromUtf8("tab_3"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.tab_3)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        spacerItem3 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem3)
        self.progressBar = QtGui.QProgressBar(self.tab_3)
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName(_fromUtf8("progressBar"))
        self.verticalLayout_4.addWidget(self.progressBar)
        spacerItem4 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem4)
        self.tabWidget.addTab(self.tab_3, _fromUtf8(""))
        self.tab_4 = QtGui.QWidget()
        self.tab_4.setObjectName(_fromUtf8("tab_4"))
        self.tabWidget.addTab(self.tab_4, _fromUtf8(""))
        self.verticalLayout_3.addWidget(self.tabWidget)
        ReplayFrameskipMainWindow.setCentralWidget(self.centralwidget)
        self.actionCascade = QtGui.QAction(ReplayFrameskipMainWindow)
        self.actionCascade.setObjectName(_fromUtf8("actionCascade"))
        self.actionTile = QtGui.QAction(ReplayFrameskipMainWindow)
        self.actionTile.setObjectName(_fromUtf8("actionTile"))

        self.retranslateUi(ReplayFrameskipMainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(ReplayFrameskipMainWindow)

    def retranslateUi(self, ReplayFrameskipMainWindow):
        ReplayFrameskipMainWindow.setWindowTitle(_translate("ReplayFrameskipMainWindow", "replayComp", None))
        self.label.setText(_translate("ReplayFrameskipMainWindow", "Input file:", None))
        self.toolButton.setText(_translate("ReplayFrameskipMainWindow", "...", None))
        self.label_2.setText(_translate("ReplayFrameskipMainWindow", "Output file:", None))
        self.outputFile.setText(_translate("ReplayFrameskipMainWindow", "/home/lmanso/floor2.rpl.2", None))
        self.toolButton_2.setText(_translate("ReplayFrameskipMainWindow", "...", None))
        self.label_3.setText(_translate("ReplayFrameskipMainWindow", "Measures per second", None))
        self.fileButton.setText(_translate("ReplayFrameskipMainWindow", "Next", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("ReplayFrameskipMainWindow", "File selection", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("ReplayFrameskipMainWindow", "Processing...", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("ReplayFrameskipMainWindow", "Done", None))
        self.actionCascade.setText(_translate("ReplayFrameskipMainWindow", "Cascade windows", None))
        self.actionTile.setText(_translate("ReplayFrameskipMainWindow", "Tile windows", None))

