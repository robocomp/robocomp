import datetime
import sys
from string import Template

from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice, get_name_number, IDSLPool
from robocompdsl.templates.common.templatedict import TemplateDict


class src_main_py(TemplateDict):
    def __init__(self, component):
        super(src_main_py, self).__init__()
        self.component = component
        self['import_qtwidgets'] = self.import_qtwidgets()
        self['app_creation'] = self.app_creation()

    def import_qtwidgets(self):
        result = ""
        if self.component.gui is not None:
            result += 'from PySide2 import QtWidgets\n'
        return result

    def app_creation(self):
        result = ""
        if self.component.gui is not None:
            result += 'app = QtWidgets.QApplication(sys.argv)\n'
        else:
            result += 'app = QtCore.QCoreApplication(sys.argv)\n'
        return result
