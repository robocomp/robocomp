import datetime
from string import Template

from robocompdsl.dsl_parsers.dsl_factory import DSLFactory
from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice, IDSLPool, get_name_number
from robocompdsl.templates.common.templatedict import TemplateDict
from robocompdsl.templates.templatePython.plugins.base.functions import function_utils as utils


GUI_IMPORT_STR = """
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
"""


class src_genericworker_py(TemplateDict):
    def __init__(self, component):
        super(src_genericworker_py, self).__init__()
        self.component = component
        self['ui_import'] = self.ui_import()
        self['qt_class_type'] = self.qt_class_type()
        self['gui_setup'] = self.gui_setup()


    def qt_class_type(self):
        if self.component.gui is not None:
            inherit_from = 'QtWidgets.' + self.component.gui[1]
        else:
            inherit_from = 'QtCore.QObject'
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
