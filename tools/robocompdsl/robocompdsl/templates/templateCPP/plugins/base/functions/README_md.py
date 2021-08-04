from templates.common.templatedict import TemplateDict


class README_md(TemplateDict):
    def __init__(self, component):
        super(README_md, self).__init__()
        self.component = component
        self['component_name'] = self.component.name

