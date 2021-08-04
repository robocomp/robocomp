from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict



class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['component_name'] = self.component.name
        self['interface_sources'] = self.interface_sources()
        self['wrap_ice'] = self.wrap_ice()

    def interface_sources(self):
        result = ""
        # TODO: refactor in one loop
        for ima in self.component.implements:
            if type(ima) == str:
                im = ima
            else:
                im = ima[0]
            if communication_is_ice(ima):
                result += im.lower() + 'I.cpp\n'

        for subscribe in self.component.subscribesTo:
            interface_name = subscribe.name
            if communication_is_ice(subscribe):
                result += interface_name.lower() + 'I.cpp\n'
        return result


    def wrap_ice(self):
        interface_names = []
        for im in sorted(self.component.recursiveImports + self.component.ice_interfaces_names):
            name = im.split('/')[-1].split('.')[0]
            interface_names.append(name)

        result = "ROBOCOMP_IDSL_TO_ICE( CommonBehavior "
        result += ' '.join(interface_names)
        result += ")\n"
        result += "ROBOCOMP_ICE_TO_SRC( CommonBehavior "
        result += ' '.join(interface_names)
        result += ")\n"
        return result

