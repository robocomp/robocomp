from templates.common.templatedict import TemplateDict


class CMakeLists_txt(TemplateDict):
    def __init__(self, component):
        super(CMakeLists_txt, self).__init__()
        self.component = component
        if self.component.gui is not None:
            wrap_python_ui = "WRAP_PYTHON_UI( mainUI )"
        else:
            wrap_python_ui = ""
        self['component_name'] = self.component.name
        self['wrap_python_ui'] = wrap_python_ui

