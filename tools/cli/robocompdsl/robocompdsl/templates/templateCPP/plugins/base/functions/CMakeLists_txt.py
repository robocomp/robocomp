from robocompdsl.templates.common.templatedict import TemplateDict


class CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super(CMakeLists_txt, self).__init__()
        self.component = component
        self['component_name'] = self.component.name

