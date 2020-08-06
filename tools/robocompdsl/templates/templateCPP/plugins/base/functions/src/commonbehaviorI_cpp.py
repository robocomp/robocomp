from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict
import datetime


class src_commonbehaviorI_cpp(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        if self.component.language.lower() == 'cpp':
            const = "const"
            ampersand = "&"
        else:
            const = ""
            ampersand = ""
        self['year'] = str(datetime.date.today().year)
        self['const'] = const
        self['ampersand'] = ampersand

