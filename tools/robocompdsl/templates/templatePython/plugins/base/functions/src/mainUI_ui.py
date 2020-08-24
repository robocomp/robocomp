from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict


class src_mainUI_ui(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['gui_type'] = self.component.gui.widget
        self['component_name'] = self.component.name

