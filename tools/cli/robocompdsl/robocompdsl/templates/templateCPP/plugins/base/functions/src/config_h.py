from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice
from robocompdsl.templates.common.templatedict import TemplateDict


class src_config_h(TemplateDict):
    def __init__(self, component):
        super(src_config_h, self).__init__()
        self.component = component
        self['component_name'] = self.component.name


