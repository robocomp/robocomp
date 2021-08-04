from templates.common.templatedict import TemplateDict


class DoxyFile(TemplateDict):
    def __init__(self, component):
        super(DoxyFile, self).__init__()
        self.component = component
        self['component_name'] = self.component.name

