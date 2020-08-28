from ..common.abstracttemplatesmanager import InterfaceTemplateManager
from ..common.plugin_collection import PluginCollection
from . import plugins


class TemplateManagerIce(InterfaceTemplateManager):
    def __init__(self, module):
        self.files = {
                'regular': [
                    'TEMPLATE.ICE'
                ],
                'template_path': "templateICE/files/"
        }
        current_plugins = PluginCollection(plugins.__name__)
        super(TemplateManagerIce, self).__init__(module, current_plugins)
