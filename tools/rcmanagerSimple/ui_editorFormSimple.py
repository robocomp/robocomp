# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'editorFormSimple.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(768, 539)
        self.table = QTableWidget(Form)
        if (self.table.columnCount() < 7):
            self.table.setColumnCount(7)
        __qtablewidgetitem = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(4, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(5, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.table.setHorizontalHeaderItem(6, __qtablewidgetitem6)
        self.table.setObjectName(u"table")
        self.table.setGeometry(QRect(4, 170, 761, 361))
        self.table.setSortingEnabled(False)
        self.groupBox_2 = QGroupBox(Form)
        self.groupBox_2.setObjectName(u"groupBox_2")
        self.groupBox_2.setGeometry(QRect(650, 10, 111, 81))
        self.newButton = QPushButton(self.groupBox_2)
        self.newButton.setObjectName(u"newButton")
        self.newButton.setGeometry(QRect(10, 20, 91, 21))
        self.deleteButton = QPushButton(self.groupBox_2)
        self.deleteButton.setObjectName(u"deleteButton")
        self.deleteButton.setGeometry(QRect(10, 45, 91, 21))
        self.groupBox = QGroupBox(Form)
        self.groupBox.setObjectName(u"groupBox")
        self.groupBox.setGeometry(QRect(650, 90, 111, 71))
        self.readButton = QPushButton(self.groupBox)
        self.readButton.setObjectName(u"readButton")
        self.readButton.setGeometry(QRect(10, 18, 91, 21))
        self.writeButton = QPushButton(self.groupBox)
        self.writeButton.setObjectName(u"writeButton")
        self.writeButton.setGeometry(QRect(10, 43, 91, 20))
        self.frame = QFrame(Form)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(0, 0, 641, 161))
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.upLine = QLineEdit(self.frame)
        self.upLine.setObjectName(u"upLine")
        self.upLine.setGeometry(QRect(10, 80, 300, 25))
        self.upLine.setReadOnly(False)
        self.pathButton = QPushButton(self.frame)
        self.pathButton.setObjectName(u"pathButton")
        self.pathButton.setGeometry(QRect(555, 30, 75, 25))
        self.label_2 = QLabel(self.frame)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 10, 56, 17))
        self.label_6 = QLabel(self.frame)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(330, 60, 121, 17))
        self.downLine = QLineEdit(self.frame)
        self.downLine.setObjectName(u"downLine")
        self.downLine.setGeometry(QRect(330, 80, 300, 25))
        self.downLine.setReadOnly(False)
        self.label_5 = QLabel(self.frame)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(10, 60, 121, 17))
        self.pathLine = QLineEdit(self.frame)
        self.pathLine.setObjectName(u"pathLine")
        self.pathLine.setGeometry(QRect(230, 30, 311, 25))
        self.pathLine.setReadOnly(False)
        self.aliasLine = QLineEdit(self.frame)
        self.aliasLine.setObjectName(u"aliasLine")
        self.aliasLine.setGeometry(QRect(10, 30, 211, 25))
        self.aliasLine.setReadOnly(False)
        self.endpointLine = QLineEdit(self.frame)
        self.endpointLine.setObjectName(u"endpointLine")
        self.endpointLine.setGeometry(QRect(220, 130, 190, 25))
        self.endpointLine.setReadOnly(False)
        self.label_3 = QLabel(self.frame)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(220, 110, 56, 21))
        self.label_4 = QLabel(self.frame)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(230, 10, 56, 17))
        self.dependencesLine = QLineEdit(self.frame)
        self.dependencesLine.setObjectName(u"dependencesLine")
        self.dependencesLine.setGeometry(QRect(10, 130, 200, 25))
        self.dependencesLine.setReadOnly(False)
        self.label_7 = QLabel(self.frame)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setGeometry(QRect(10, 110, 91, 21))
        self.configLine = QLineEdit(self.frame)
        self.configLine.setObjectName(u"configLine")
        self.configLine.setGeometry(QRect(420, 130, 211, 25))
        self.configLine.setReadOnly(False)
        self.label_8 = QLabel(self.frame)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setGeometry(QRect(420, 110, 111, 21))

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"managerComp file editor", None))
        ___qtablewidgetitem = self.table.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("Form", u"Alias", None));
        ___qtablewidgetitem1 = self.table.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("Form", u"Deps", None));
        ___qtablewidgetitem2 = self.table.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("Form", u"Endpoint", None));
        ___qtablewidgetitem3 = self.table.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("Form", u"Path", None));
        ___qtablewidgetitem4 = self.table.horizontalHeaderItem(4)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("Form", u"Up Command", None));
        ___qtablewidgetitem5 = self.table.horizontalHeaderItem(5)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("Form", u"Down Command", None));
        ___qtablewidgetitem6 = self.table.horizontalHeaderItem(6)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("Form", u"Config", None));
        self.groupBox_2.setTitle(QCoreApplication.translate("Form", u"Components", None))
        self.newButton.setText(QCoreApplication.translate("Form", u"New", None))
        self.deleteButton.setText(QCoreApplication.translate("Form", u"Delete", None))
        self.groupBox.setTitle(QCoreApplication.translate("Form", u"File", None))
        self.readButton.setText(QCoreApplication.translate("Form", u"Read", None))
        self.writeButton.setText(QCoreApplication.translate("Form", u"Write", None))
        self.pathButton.setText(QCoreApplication.translate("Form", u" browse", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"Alias:", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"Down Command:", None))
        self.label_5.setText(QCoreApplication.translate("Form", u"Up Command:", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"Endpoint", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"Path:", None))
        self.label_7.setText(QCoreApplication.translate("Form", u"Dependences", None))
        self.label_8.setText(QCoreApplication.translate("Form", u"Configuration file", None))
    # retranslateUi

