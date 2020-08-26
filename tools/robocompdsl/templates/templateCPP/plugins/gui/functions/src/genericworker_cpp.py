import datetime
from string import Template
from dsl_parsers.parsing_utils import get_name_number, communication_is_ice
from templates.common.templatedict import TemplateDict

GUI_SETUP_STR = """
#ifdef USE_QTGUI
	setupUi(this);
	show();
#endif
"""


class genericworker_cpp(TemplateDict):
    def __init__(self, component):
        super(genericworker_cpp, self).__init__()
        self.component = component
        self['gui_setup'] = self.gui_setup()

    def gui_setup(self):
        result = ""
        if self.component.gui is not None:
            result += GUI_SETUP_STR
        return result

