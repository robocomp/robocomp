from dsl_parsers.parsing_utils import communication_is_ice
from templates.common.templatedict import TemplateDict

STATEMACHINE_VISUAL_SOURCES_STR = """
$ENV{ROBOCOMP}/classes/statemachinewidget/edge.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/node.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/graphwidget.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/qstateMachineWrapper.cpp
"""



CPP11_ICE_STR = """
ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)
"""


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['component_name'] = self.component.name
        self['interface_sources'] = self.interface_sources()
        self['statemachine_visual_sources'] = self.statemachine_visual_sources()
        self['cpp11_ice_packages'] = self.cpp11_ice_packages()
        self['wrap_ice'] = self.wrap_ice()
        self['wrap_ui'] = self.wrap_ui()

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

    def statemachine_visual_sources(self):
        result = ""
        if self.component.statemachine_visual:
            result += STATEMACHINE_VISUAL_SOURCES_STR
        return result


    def cpp11_ice_packages(self):
        result = ""
        if self.component.language.lower() == "cpp11":
            result += CPP11_ICE_STR
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

    def wrap_ui(self):
        result = ""
        if self.component.gui is not None:
            result += "QT_WRAP_UI( UI_HEADERS mainUI.ui )\n"
        return result
