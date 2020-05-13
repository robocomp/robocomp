from ..common.abstracttemplate import CustomTemplate, AbstractInterfaceTemplate


class TemplateIce(AbstractInterfaceTemplate):
    def __init__(self, module):
        self.files = {
                'regular': [
                    'TEMPLATE.ICE'
                ],
                'template_path': "templateICE/files/"
        }
        super().__init__(module)
