from robocompdsl.dsl_parsers.parsing_utils import communication_is_ice
from robocompdsl.templates.common.templatedict import TemplateDict

DSR_FIND_EIGEN = """\
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
"""

DSR_DEDINITIONS = """\
add_definitions(-g  -fmax-errors=1 -std=c++2a -fno-char8_t)
"""

DSR_LIBS = " dsr_core dsr_gui dsr_api fastcdr fastrtps osgDB OpenThreads Eigen3::Eigen"


class src_CMakeListsSpecific_txt(TemplateDict):
    def __init__(self, component):
        super(src_CMakeListsSpecific_txt, self).__init__()
        dsr_find_eigen = ""
        dsr_definitions = ""
        dsr_libs = ""
        if component.dsr:
            dsr_find_eigen = DSR_FIND_EIGEN
            dsr_definitions = DSR_DEDINITIONS
            dsr_libs = DSR_LIBS
        self['dsr_find_eigen'] = dsr_find_eigen
        self['dsr_definitions'] = dsr_definitions
        self['dsr_libs'] = dsr_libs
