from string import Template

from dsl_parsers.parsing_utils import communication_is_ice, get_name_number
from templates.common.templatedict import TemplateDict

STORM_TOPIC_MANAGER_STR = """\
# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
"""

class etc_config(TemplateDict):
    def __init__(self, component):
        super().__init__()
        self.component = component
        self['config_implements_endpoints'] = self.config_implements_endpoints()
        self['config_subscribes_endpoints'] = self.config_subscribes_endpoints()
        self['config_requires_proxies'] = self.config_requires_proxies()
        self['storm_topic_manager'] = self.storm_topic_manager()

    def config_implements_endpoints(self):
        result = ""
        for interface in self.component.implements:
            if communication_is_ice(interface):
                result += interface.name + ".Endpoints=tcp -p 0\n"
        if result != "":
            result = '# Endpoints for implements interfaces\n' + result + '\n\n'
        return result

    def config_subscribes_endpoints(self):
        result = ""
        for interface in self.component.subscribesTo:
            if communication_is_ice(interface):
                result += interface.name + "Topic.Endpoints=tcp -p 0\n"
        if result != "":
            result = '# Endpoints for subscriptions interfaces\n' + result + '\n\n'
        return result

    def config_requires_proxies(self):
        result = ""
        for interface, num in get_name_number(self.component.requires):
            if communication_is_ice(interface):
                port = 0
                if interface.name == 'DifferentialRobot': port = 10004
                if interface.name == 'Laser': port = 10003
                result += interface.name + num + "Proxy = " + interface.name.lower() + ":tcp -h localhost -p " + str(
                    port) + "\n"
        if result != "":
            result = '# Proxies for required interfaces\n' + result + '\n\n'
        return result

    def storm_topic_manager(self):
        result = ""
        if len(self.component.publishes + self.component.subscribesTo) > 0:
            result += STORM_TOPIC_MANAGER_STR
        return result

