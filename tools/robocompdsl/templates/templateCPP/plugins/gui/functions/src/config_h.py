from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict


class src_config_h(TemplateDict):
    def __init__(self, component):
        super(src_config_h, self).__init__()
        self.component = component
        need_gui = "// Comment out this line if your application has a QtGui\n"
        if self.component.gui is not None:
            need_gui += "#define USE_QTGUI\n\n"
        else:
            need_gui += "#define USE_QTGUI\n\n"
        self['need_gui'] = need_gui


