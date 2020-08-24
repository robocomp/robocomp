from templates.common.templatedict import TemplateDict

AGM_INCLUDES_STR = """
# AGM Agent\'s requirements
find_package(LibXml2 REQUIRED)
include_directories(LIBXML2_INCLUDE_DIRS)
include_directories(/usr/include/libxml2/)
"""


class src_CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super(src_CMakeLists_txt, self).__init__()
        self.component = component
        self['agm_includes'] = self.agm_includes()


    def agm_includes(self):
        result = ""
        if 'agmagent' in [x.lower() for x in self.component.options]:
            result += AGM_INCLUDES_STR
        if self.component.is_agm1_agent():
            result += 'SET(LIBS ${LIBS} -lagm)\n'
            result += 'ADD_DEFINITIONS( -I/usr/include/libxml2/)\n'
        if self.component.is_agm2_agent():
            result += 'SET(LIBS ${LIBS} -lagm2)\n'
            result += 'ADD_DEFINITIONS( -I/usr/include/libxml2/)\n'
            result += 'include(/usr/local/share/cmake/FindAGM2.cmake)\n'

        return result

