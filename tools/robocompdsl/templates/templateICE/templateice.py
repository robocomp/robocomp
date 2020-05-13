from ..common.abstracttemplatesmanager import InterfaceTemplateManager


class TemplateManagerIce(InterfaceTemplateManager):
    def __init__(self, module):
        self.files = {
                'regular': [
                    'TEMPLATE.ICE'
                ],
                'template_path': "templateICE/files/"
        }
        super().__init__(module)
