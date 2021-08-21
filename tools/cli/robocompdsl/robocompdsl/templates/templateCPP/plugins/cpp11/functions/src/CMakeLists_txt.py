from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice
from robocompdsl.templates.common.templatedict import TemplateDict


CPP11_ICE_STR = """
ADD_DEFINITIONS ("-DICE_CPP11_MAPPING")
FIND_PACKAGE( Ice REQUIRED COMPONENTS Ice++11 IceStorm++11)
"""


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['cpp11_ice_packages'] = self.cpp11_ice_packages()

    def cpp11_ice_packages(self):
        result = ""
        if self.component.language.lower() == "cpp11":
            result += CPP11_ICE_STR
        return result

