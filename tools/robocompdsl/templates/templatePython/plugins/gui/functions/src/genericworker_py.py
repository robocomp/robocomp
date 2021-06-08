import datetime
from string import Template

from dsl_parsers.dsl_factory import DSLFactory
from dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from templates.common.templatedict import TemplateDict
from templates.templatePython.plugins.base.functions import function_utils as utils


GUI_IMPORT_STR = """
from PySide2 import QtWidgets, QtCore
try:
    from ui_mainUI import *
except:
    print("Can\'t import UI file. Did you run \'make\'?")
    sys.exit(-1)
"""

GUI_SETUP_STR = """
self.ui = Ui_guiDlg()
self.ui.setupUi(self)
self.show()

self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
self.Period = 30
self.timer = QtCore.QTimer(self)
"""



KILL_YOURSELF_STR = """
# Trash never used but kept asked by pbustos: "Will be removed in the next robocompdsl release." 20210608.
@QtCore.Slot()
def killYourSelf(self):
    rDebug("Killing myself")
    self.kill.emit()
"""

SET_PERIOD_STR = """
# Trash never used but kept asked by pbustos: "Will be removed in the next robocompdsl release." 20210608.
# \\brief Change compute period
# @param per Period in ms
@QtCore.Slot(int)
def setPeriod(self, p):
    print("Period changed", p)
    self.Period = p
    self.timer.start(self.Period)
"""


class src_genericworker_py(TemplateDict):
    def __init__(self, component):
        super(src_genericworker_py, self).__init__()
        self.component = component
        self['ui_import'] = self.ui_import()
        self['qt_class_type'] = self.qt_class_type()
        self['gui_setup'] = self.gui_setup()
        self['kill_yourself_method'] = self.kill_yourself_method()
        self['set_period_method'] = self.set_period_method()
        self['qt_kill_signal'] = self.qt_kill_signal()




    def qt_class_type(self):
        if self.component.gui is not None:
            inherit_from = 'QtWidgets.' + self.component.gui[1]
        else:
            inherit_from = ''
        return inherit_from

    def gui_setup(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_SETUP_STR
        return result

    def ui_import(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_IMPORT_STR
        return result

    def kill_yourself_method(self):
        result = ""
        if self.component.gui is not None:
            result += KILL_YOURSELF_STR
        return result

    def set_period_method(self):
        result = ""
        if self.component.gui is not None:
            result += SET_PERIOD_STR
        return result

    def qt_kill_signal(self):
        result = ""
        if self.component.gui is not None:
            result += "kill = QtCore.Signal()\n"
        return result
