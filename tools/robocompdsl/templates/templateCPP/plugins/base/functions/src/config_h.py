from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        need_gui = ""
        if self.component.gui is not None:
            need_gui = "#define USE_QTGUI\n\n"

        self['component_name'] = self.component.name
        self['need_gui'] = need_gui

