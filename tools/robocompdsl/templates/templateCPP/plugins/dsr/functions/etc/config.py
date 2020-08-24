from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from templates.common.templatedict import TemplateDict


DSR_CONFIG_STR = """\
agent_id = 0 # Change id
agent_name = ${name}
tree_view = true
graph_view = true
2d_view = true
3d_view = true
#Ice.MessageSizeMax=20004800
"""

class etc_config(TemplateDict):
    def __init__(self, component):
        super(etc_config, self).__init__()
        self.component = component
        self['dsr_config'] = self.dsr_config()

    def dsr_config(self):
        result = ""
        if self.component.dsr:
            result += Template(DSR_CONFIG_STR).substitute(name=self.component.name)
        return result

