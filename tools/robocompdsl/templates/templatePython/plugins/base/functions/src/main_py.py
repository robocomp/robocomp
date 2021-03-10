import datetime
import sys
from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number, IDSLPool
from templates.common.templatedict import TemplateDict

class src_main_py(TemplateDict):
    def __init__(self, component):
        super(src_main_py, self).__init__()
        self.component = component
        self['year'] = str(datetime.date.today().year)
        self['component_name'] = self.component.name

