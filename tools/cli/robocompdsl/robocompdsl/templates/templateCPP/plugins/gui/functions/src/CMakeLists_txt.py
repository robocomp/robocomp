from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice
from robocompdsl.templates.common.templatedict import TemplateDict



class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['wrap_ui'] = self.wrap_ui()

    def wrap_ui(self):
        result = ""
        if self.component.gui is not None:
            result += "QT_WRAP_UI( UI_HEADERS mainUI.ui )\n"
        return result
