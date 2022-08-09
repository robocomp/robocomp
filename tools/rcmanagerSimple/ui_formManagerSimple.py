# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'formManagerSimple.ui'
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
        Form.resize(387, 518)
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        Form.setMinimumSize(QSize(200, 250))
        self.verticalLayout_4 = QVBoxLayout(Form)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setSizeConstraint(QLayout.SetNoConstraint)
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setSizeConstraint(QLayout.SetNoConstraint)
        self.tabWidget = QTabWidget(Form)
        self.tabWidget.setObjectName(u"tabWidget")
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy1.setHorizontalStretch(1)
        sizePolicy1.setVerticalStretch(1)
        sizePolicy1.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy1)
        self.listTab = QWidget()
        self.listTab.setObjectName(u"listTab")
        sizePolicy1.setHeightForWidth(self.listTab.sizePolicy().hasHeightForWidth())
        self.listTab.setSizePolicy(sizePolicy1)
        self.verticalLayout_2 = QVBoxLayout(self.listTab)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setSizeConstraint(QLayout.SetNoConstraint)
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.checkList = QListWidget(self.listTab)
        self.checkList.setObjectName(u"checkList")
        sizePolicy1.setHeightForWidth(self.checkList.sizePolicy().hasHeightForWidth())
        self.checkList.setSizePolicy(sizePolicy1)

        self.verticalLayout.addWidget(self.checkList)

        self.formLayout_7 = QFormLayout()
        self.formLayout_7.setObjectName(u"formLayout_7")
        self.formLayout_7.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_7.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.formLayout_6 = QFormLayout()
        self.formLayout_6.setObjectName(u"formLayout_6")
        self.formLayout_6.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label_3 = QLabel(self.listTab)
        self.label_3.setObjectName(u"label_3")

        self.formLayout_6.setWidget(0, QFormLayout.LabelRole, self.label_3)

        self.downButton = QPushButton(self.listTab)
        self.downButton.setObjectName(u"downButton")
        icon = QIcon()
        icon.addFile(u"../../../../../../../../../../../../opt/managerComp/share/down.png", QSize(), QIcon.Normal, QIcon.Off)
        self.downButton.setIcon(icon)

        self.formLayout_6.setWidget(0, QFormLayout.FieldRole, self.downButton)


        self.formLayout_7.setLayout(0, QFormLayout.LabelRole, self.formLayout_6)

        self.downEdit = QLabel(self.listTab)
        self.downEdit.setObjectName(u"downEdit")

        self.formLayout_7.setWidget(0, QFormLayout.FieldRole, self.downEdit)


        self.verticalLayout.addLayout(self.formLayout_7)

        self.formLayout_3 = QFormLayout()
        self.formLayout_3.setObjectName(u"formLayout_3")
        self.formLayout_3.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_3.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label_5 = QLabel(self.listTab)
        self.label_5.setObjectName(u"label_5")
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)

        self.formLayout_3.setWidget(0, QFormLayout.LabelRole, self.label_5)

        self.cfgEdit = QLabel(self.listTab)
        self.cfgEdit.setObjectName(u"cfgEdit")
        sizePolicy.setHeightForWidth(self.cfgEdit.sizePolicy().hasHeightForWidth())
        self.cfgEdit.setSizePolicy(sizePolicy)

        self.formLayout_3.setWidget(0, QFormLayout.FieldRole, self.cfgEdit)


        self.verticalLayout.addLayout(self.formLayout_3)

        self.formLayout_2 = QFormLayout()
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.formLayout_2.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_2.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label_4 = QLabel(self.listTab)
        self.label_4.setObjectName(u"label_4")
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)

        self.formLayout_2.setWidget(0, QFormLayout.LabelRole, self.label_4)

        self.wdEdit = QLabel(self.listTab)
        self.wdEdit.setObjectName(u"wdEdit")
        sizePolicy.setHeightForWidth(self.wdEdit.sizePolicy().hasHeightForWidth())
        self.wdEdit.setSizePolicy(sizePolicy)

        self.formLayout_2.setWidget(0, QFormLayout.FieldRole, self.wdEdit)


        self.verticalLayout.addLayout(self.formLayout_2)

        self.formLayout_5 = QFormLayout()
        self.formLayout_5.setObjectName(u"formLayout_5")
        self.formLayout_5.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_5.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.formLayout_4 = QFormLayout()
        self.formLayout_4.setObjectName(u"formLayout_4")
        self.formLayout_4.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_4.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label_2 = QLabel(self.listTab)
        self.label_2.setObjectName(u"label_2")

        self.formLayout_4.setWidget(0, QFormLayout.LabelRole, self.label_2)

        self.upButton = QPushButton(self.listTab)
        self.upButton.setObjectName(u"upButton")
        sizePolicy2 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.upButton.sizePolicy().hasHeightForWidth())
        self.upButton.setSizePolicy(sizePolicy2)
        icon1 = QIcon()
        icon1.addFile(u"../../../../../../../../../../../../opt/rcmanager/share/up.png", QSize(), QIcon.Normal, QIcon.Off)
        self.upButton.setIcon(icon1)

        self.formLayout_4.setWidget(0, QFormLayout.FieldRole, self.upButton)


        self.formLayout_5.setLayout(0, QFormLayout.LabelRole, self.formLayout_4)

        self.upEdit = QLabel(self.listTab)
        self.upEdit.setObjectName(u"upEdit")
        sizePolicy.setHeightForWidth(self.upEdit.sizePolicy().hasHeightForWidth())
        self.upEdit.setSizePolicy(sizePolicy)

        self.formLayout_5.setWidget(0, QFormLayout.FieldRole, self.upEdit)


        self.verticalLayout.addLayout(self.formLayout_5)

        self.formLayout_8 = QFormLayout()
        self.formLayout_8.setObjectName(u"formLayout_8")
        self.formLayout_8.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_8.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.formLayout_9 = QFormLayout()
        self.formLayout_9.setObjectName(u"formLayout_9")
        self.formLayout_9.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout_9.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label_6 = QLabel(self.listTab)
        self.label_6.setObjectName(u"label_6")

        self.formLayout_9.setWidget(0, QFormLayout.LabelRole, self.label_6)

        self.restartButton = QPushButton(self.listTab)
        self.restartButton.setObjectName(u"restartButton")
        sizePolicy2.setHeightForWidth(self.restartButton.sizePolicy().hasHeightForWidth())
        self.restartButton.setSizePolicy(sizePolicy2)
        self.restartButton.setIcon(icon1)

        self.formLayout_9.setWidget(0, QFormLayout.FieldRole, self.restartButton)


        self.formLayout_8.setLayout(0, QFormLayout.LabelRole, self.formLayout_9)


        self.verticalLayout.addLayout(self.formLayout_8)

        self.formLayout = QFormLayout()
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setSizeConstraint(QLayout.SetNoConstraint)
        self.formLayout.setFieldGrowthPolicy(QFormLayout.FieldsStayAtSizeHint)
        self.label = QLabel(self.listTab)
        self.label.setObjectName(u"label")

        self.formLayout.setWidget(1, QFormLayout.LabelRole, self.label)

        self.checkEdit = QLabel(self.listTab)
        self.checkEdit.setObjectName(u"checkEdit")
        sizePolicy.setHeightForWidth(self.checkEdit.sizePolicy().hasHeightForWidth())
        self.checkEdit.setSizePolicy(sizePolicy)

        self.formLayout.setWidget(1, QFormLayout.FieldRole, self.checkEdit)


        self.verticalLayout.addLayout(self.formLayout)

        self.outputText = QTextEdit(self.listTab)
        self.outputText.setObjectName(u"outputText")
        sizePolicy1.setHeightForWidth(self.outputText.sizePolicy().hasHeightForWidth())
        self.outputText.setSizePolicy(sizePolicy1)
        self.outputText.setTabChangesFocus(True)
        self.outputText.setUndoRedoEnabled(False)
        self.outputText.setReadOnly(True)

        self.verticalLayout.addWidget(self.outputText)


        self.verticalLayout_2.addLayout(self.verticalLayout)

        self.tabWidget.addTab(self.listTab, "")
        self.graphTab = QWidget()
        self.graphTab.setObjectName(u"graphTab")
        sizePolicy1.setHeightForWidth(self.graphTab.sizePolicy().hasHeightForWidth())
        self.graphTab.setSizePolicy(sizePolicy1)
        self.verticalLayout_5 = QVBoxLayout(self.graphTab)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setSizeConstraint(QLayout.SetNoConstraint)
        self.tabWidget.addTab(self.graphTab, "")

        self.verticalLayout_3.addWidget(self.tabWidget)


        self.verticalLayout_4.addLayout(self.verticalLayout_3)


        self.retranslateUi(Form)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"RCManager", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"Comp-down:", None))
        self.downButton.setText("")
        self.downEdit.setText(QCoreApplication.translate("Form", u"-", None))
        self.label_5.setText(QCoreApplication.translate("Form", u"Configuration:", None))
        self.cfgEdit.setText(QCoreApplication.translate("Form", u"-", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"WorkingDir:", None))
        self.wdEdit.setText(QCoreApplication.translate("Form", u"-", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"Comp-up:", None))
        self.upButton.setText("")
        self.upEdit.setText(QCoreApplication.translate("Form", u"-", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"Restart", None))
        self.restartButton.setText("")
        self.label.setText(QCoreApplication.translate("Form", u"Endpoint:", None))
        self.checkEdit.setText(QCoreApplication.translate("Form", u"-", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.listTab), QCoreApplication.translate("Form", u"List view", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.graphTab), QCoreApplication.translate("Form", u"Graph view", None))
    # retranslateUi

