from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super(src_CMakeLists_txt, self).__init__()
        self.component = component
        interface_names = []
        for im in sorted(self.component.recursiveImports + self.component.ice_interfaces_names):
            name = im.split('/')[-1].split('.')[0]
            interface_names.append(name)
        self['ifaces_list'] = ' '.join(interface_names)
        self['component_name'] = self.component.name
