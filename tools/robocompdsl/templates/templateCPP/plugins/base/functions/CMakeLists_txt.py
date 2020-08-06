from templates.common.templatedict import TemplateDict


class CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['component_name'] = self.component.name

