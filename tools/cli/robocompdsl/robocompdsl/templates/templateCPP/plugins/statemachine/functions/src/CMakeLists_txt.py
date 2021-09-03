from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice
from robocompdsl.templates.common.templatedict import TemplateDict

STATEMACHINE_VISUAL_SOURCES_STR = """
$ENV{ROBOCOMP}/classes/statemachinewidget/edge.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/node.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/graphwidget.cpp
$ENV{ROBOCOMP}/classes/statemachinewidget/qstateMachineWrapper.cpp
"""


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['statemachine_visual_sources'] = self.statemachine_visual_sources()


    def statemachine_visual_sources(self):
        result = ""
        if self.component.statemachine_visual:
            result += STATEMACHINE_VISUAL_SOURCES_STR
        return result

